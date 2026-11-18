#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_dsp/juce_dsp.h>
#include <BinaryData.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <atomic>
using namespace juce;
constexpr double pi=3.14159265358979323846;
struct HarmonicColour {
 struct Row { float f,l; std::array<float,19> a{},b{}; };
 std::vector<Row> rows[2];
 void load(const char* data,int size,int bank) {
  auto lines=StringArray::fromLines(String::fromUTF8(data,size));
  auto h=StringArray::fromTokens(lines[0],",","");
  for(int j=1;j<lines.size();++j){
   auto v=StringArray::fromTokens(lines[j],",","");
   if(v.size()!=h.size())continue;
   auto get=[&](String k){auto i=h.indexOf(k);return i>=0?v[i].getFloatValue():0.f;};
   if(v[h.indexOf("settled_half_window_0_1db")]!="True")continue;
   if(bank && get("deess")!=0)continue;
   Row r; r.f=get("hz");r.l=get("input_dbu");
   for(int k=2;k<=20;++k){
    auto key="k"+String(k);float amp=std::pow(10.f,get(key+"_dbc")/20);
    float phase=(get(key+"_phase_deg")-k*get("phase_deg"))*pi/180;
    r.a[k-2]=std::copysign(amp,std::cos(phase));r.b[k-2]=0;
   }rows[bank].push_back(r);
  }
 }
 HarmonicColour(){for(int i=0;i<BinaryData::namedResourceListSize;++i){int n=0;auto d=BinaryData::getNamedResource(BinaryData::namedResourceList[i],n);if(d)load(d,n,String(BinaryData::namedResourceList[i]).contains("mod8")?1:0);}}
 std::array<float,19> coeff(float f,float l,float ratio,double sampleRate) const {
  std::array<float,19> out{};
  float mix=jlimit(0.f,1.f,(ratio-2)/6);
  for(int bank=0;bank<2;++bank){
   float sum=0;std::array<float,19> a{};
   for(auto& r:rows[bank]){
    float df=std::log2(jlimit(100.f,10000.f,f)/r.f);
    float dl=(jlimit(bank?-30.f:-18.f,4.f,l)-r.l)/12;
    float w=1/std::max(0.000001f,df*df+dl*dl);
    sum+=w;
    // Signed in-phase Chebyshev coefficient; quadrature phase not reproduced.
    for(int k=0;k<19;++k)a[k]+=w*r.a[k];
   }
   if(sum>0)for(int k=0;k<19;++k)out[k]+=a[k]/sum*(bank?mix:1-mix);
  }
  // Creative high-drive extension: preserve all measured profiles through +4 dBu.
  // Scale total RMS harmonic amplitude, then redistribute energy between parity groups.
  if(l>4.f){
   const double t=l<=10.f?double(l-4.f)/12.0:0.5+0.5*double(l-10.f)/8.0;
   const double blend=std::clamp(t,0.0,1.0), boost=1.0+blend;
   double even=0,odd=0;
   for(int k=2;k<=20;++k){
    if(k*double(f)>=.499*sampleRate)continue;
    double a=out[k-2]; (k%2==0?even:odd)+=a*a;
   }
   const double total=even+odd;
   if(even>1e-30 && odd>1e-30){
    const double fraction=even/total, target=fraction+blend*(.5-fraction);
    const double es=boost*std::sqrt(target*total/even), os=boost*std::sqrt((1-target)*total/odd);
    for(int k=2;k<=20;++k)out[k-2]*=float(k%2==0?es:os);
   }else{
    // At high fundamentals only one parity may be below Nyquist.
    // Preserve the intended in-band THD multiplier without folding ultrasonic orders back.
    for(auto& a:out)a*=float(boost);
   }
  }
  // Creative bass coloration, independent of the mid/high reference anchors.
  // Review scale is dBc + 4; targets at +4/+8/+10/+18 are -78/-72/-70/-68.
  // Boost only K2–K5; preserve the existing K6–K20 coefficients.
  const double bassWeight=std::clamp(1.0-std::log10(std::max(100.0,double(f))/100.0),0.0,1.0);
  if(bassWeight>0 && l>-18.f){
   double peak=0;
   for(int k=2;k<=5;++k)if(k*double(f)<.499*sampleRate)peak=std::max(peak,std::abs(double(out[k-2])));
   if(peak>1e-15){
    const double drive=double(l);
    const double targetDisplay=drive<=8.0?-78.0+6.0*std::clamp((drive-4.0)/4.0,0.0,1.0):drive<=10.0?-72.0+2.0*(drive-8.0)/2.0:-70.0+2.0*std::clamp((drive-10.0)/8.0,0.0,1.0);
    const double target=std::pow(10.0,(targetDisplay-4.0)/20.0);
    const double ramp=std::clamp((double(l)+18.0)/22.0,0.0,1.0);
    const double scale=std::pow(target/peak,bassWeight*ramp);
    for(int k=2;k<=5;++k)out[k-2]*=float(scale);
   }
  }
  // Use the actual settled 525C high-drive anchors at IN 50%, OUT 75%, stock Compress.
  // Bass remains exactly the original algorithm; crossfade smoothly over the 100 Hz--1 kHz band.
  if(l>4.f && f>100.f){
   struct High {float f,l;std::array<float,19> a;};
   static constexpr High high[]={{1000.f,10.f,{1.865900657e-5f,-7.412016437e-6f,8.861476052e-6f,-4.600028330e-6f,1.959136661e-6f,8.624753947e-7f,7.034837151e-8f,1.399852457e-6f,-1.536459735e-6f,2.082172716e-6f,1.184803576e-6f,2.180211205e-6f,1.234052097e-6f,2.287389770e-6f,-5.711306693e-7f,9.325469315e-7f,-1.450189049e-7f,-8.710349579e-7f,5.721977590e-7f}},
{1000.f,18.f,{1.406201699e-5f,-7.856830613e-6f,5.504732849e-6f,-5.524730304e-6f,-7.521950012e-6f,4.204599075e-6f,-2.801134174e-6f,2.765529641e-6f,9.629545914e-7f,-1.673698449e-6f,3.933678580e-6f,-1.825097518e-6f,3.326519203e-6f,1.247892516e-6f,-6.608091027e-7f,2.431371361e-7f,-1.699636334e-6f,1.030631230e-6f,1.894938589e-6f}},
{1000.f,24.f,{-4.921433063e-6f,1.063748937e-5f,4.432608467e-6f,-7.150918917e-6f,3.799109194e-6f,5.982315438e-6f,-3.548867234e-6f,-5.656563408e-6f,1.299350455e-6f,4.861774103e-6f,-3.089052523e-6f,-2.355232944e-6f,1.530167360e-6f,-3.640100896e-6f,-6.719616302e-7f,2.241330660e-6f,6.257727741e-7f,-2.468052456e-6f,-9.732699880e-7f}},
{5000.f,10.f,{5.819466230e-5f,-1.403725105e-5f,-1.139316921e-5f,3.932903103e-6f,3.790541439e-6f,-1.561259775e-6f,-1.423337922e-6f,-1.062980585e-6f,-1.244936208e-6f,5.106104276e-7f,-1.164444287e-6f,3.572219366e-7f,-1.040878586e-6f,-2.037678831e-7f,-6.461566004e-7f,2.202760648e-7f,3.042145545e-7f,4.323034042e-7f,-2.040674376e-7f}},
{5000.f,18.f,{4.288467057e-5f,-1.375673038e-5f,-1.878842079e-5f,4.920769200e-6f,-8.743883600e-6f,-1.178268771e-6f,3.101037820e-6f,-1.946441912e-6f,3.231394910e-6f,-2.070830901e-6f,-3.159737017e-6f,-1.808246441e-6f,-1.525843918e-6f,-1.696562521e-6f,1.092357128e-6f,-9.787940923e-7f,8.407625916e-7f,1.277215492e-6f,-8.795796889e-7f}}};
   auto measured=[&](float freq){
    // Evaluate the old measured +4 reference without creative extrapolation.
    std::array<float,19> base{};
    for(int bank=0;bank<2;++bank){double sum=0;std::array<double,19> acc{};
     for(const auto& r:rows[bank]){const double df=std::log2(freq/r.f),dl=(4.-r.l)/12.;const double w=1./std::max(1e-6,df*df+dl*dl);sum+=w;for(int k=0;k<19;++k)acc[k]+=w*r.a[k];}
     if(sum>0)for(int k=0;k<19;++k)base[k]+=float(acc[k]/sum*(bank?mix:1-mix));
    }
    const High* lo=nullptr;const High* hi=nullptr;
    for(const auto& r:high)if(r.f==freq){if(r.l<=l&&(!lo||r.l>lo->l))lo=&r;if(r.l>=l&&(!hi||r.l<hi->l))hi=&r;}
    // At 10 kHz no settled high-drive source rows exist; hold +4 rather than invent a measurement.
    if(!lo&&!hi)return base;
    if(!hi)return lo->a;
    const auto& a=lo?lo->a:base;const float start=lo?lo->l:4.f;
    const float t=hi->l>start?jlimit(0.f,1.f,(l-start)/(hi->l-start)):1.f;
    for(int k=0;k<19;++k)base[k]=a[k]+t*(hi->a[k]-a[k]);
    return base;
   };
   const float low=f<=5000.f?1000.f:5000.f, upper=f<=5000.f?5000.f:10000.f;
   const auto a=measured(low),b=measured(upper);
   const float t=jlimit(0.f,1.f,float(std::log(f/low)/std::log(upper/low)));
   const float w=jlimit(0.f,1.f,float(std::log10(f/100.f)));
   for(int k=0;k<19;++k)out[k]=(1-w)*out[k]+w*(a[k]+t*(b[k]-a[k]));
  }
  return out;
 }
};
struct RippleFilter {
 double x1=0,x2=0,y1=0,y2=0,mean=0;
 double run(double x,double cosw,double pole,double alpha){
  double scale=(1-2*pole*cosw+pole*pole)/std::max(1e-10,2-2*cosw);
  double y=scale*(x-2*cosw*x1+x2)+2*pole*cosw*y1-pole*pole*y2;
  x2=x1;x1=x;y2=y1;y1=y;mean=alpha*mean+(1-alpha)*y;return std::max(0.0,mean);
 }
};
struct State {
 double hpZ1=0,hpZ2=0; int hpMode=0;
 double optoMemory=0,busMemory=0,peakEnv=0; int lastAuto=0;
 double gr=0,feedback=0,lp=0,old=0,older=0,frequency=1000,clock=0,lastCross=-1,period=44.1,cosw=0,pole=.9,inputPower=0,previousGain=1;
 int update=0; RippleFilter detector,amplitude;
 std::array<float,19> c{};
};
class Proc:public AudioProcessor {
public:
 AudioProcessorValueTreeState params;
 HarmonicColour colour;
 State st[2]; double sr=44100;
 std::atomic<float> meter{0},vu{-100},ov{0};
 static AudioProcessorValueTreeState::ParameterLayout layout(){
  AudioProcessorValueTreeState::ParameterLayout l;
  auto add=[&](const char* id,const char* name,float lo,float hi,float def,float skew=1.f){
   l.add(std::make_unique<AudioParameterFloat>(ParameterID{id,1},name,NormalisableRange<float>(lo,hi,0.f,skew),def));};
  add("out","Output",-60,10,0);add("threshold","Threshold",-20,10,0);
  add("attack","Attack",1,25,10);add("release","Release",.3,3,.5);
  add("var_attack","Variable Attack",.02f,100.f,10.f,.35f);add("var_release","Variable Release",.01f,5.f,.5f,.35f);
  l.add(std::make_unique<AudioParameterBool>(ParameterID{"variable_ar",1},"Variable A/R",false));
  add("ratio","Ratio",1,30,4,.4);
  for(auto id:{"meter","type","knee","in","colour"})l.add(std::make_unique<AudioParameterBool>(ParameterID{id,1},id,String(id)=="link"||String(id)=="in"||String(id)=="colour"));
  l.add(std::make_unique<AudioParameterChoice>(ParameterID{"thrust",1},"HPF",StringArray{"Off","80 Hz","120 Hz","250 Hz"},0));
  l.add(std::make_unique<AudioParameterChoice>(ParameterID{"mode",1},"Mode",StringArray{"Off","Detonate","Explode","Burst"},0));
  l.add(std::make_unique<AudioParameterChoice>(ParameterID{"detector",1},"Detector",StringArray{"527 RMS","Peak","Hybrid"},0));
  l.add(std::make_unique<AudioParameterChoice>(ParameterID{"auto_release",1},"Auto Release",StringArray{"Manual","VCA","OPTO","BVCA"},0));
  add("mix","Mix",0,100,100);
  add("digital_out","Digital Output",-15,15,0);
  l.add(std::make_unique<AudioParameterBool>(ParameterID{"nasty",1},"Nasty",false));
  return l;
 }
 Proc():AudioProcessor(BusesProperties().withInput("Input",AudioChannelSet::stereo(),true).withOutput("Output",AudioChannelSet::stereo(),true)),params(*this,nullptr,"PARAMS",layout()){}
 float p(const char* id){return params.getRawParameterValue(id)->load();}
 const String getName()const override{return "527";}
 bool acceptsMidi()const override{return false;}bool producesMidi()const override{return false;}bool isMidiEffect()const override{return false;}
 double getTailLengthSeconds()const override{return 0;}int getNumPrograms()override{return 1;}int getCurrentProgram()override{return 0;}void setCurrentProgram(int)override{}const String getProgramName(int)override{return {};}void changeProgramName(int,const String&)override{}
 bool isBusesLayoutSupported(const BusesLayout& b)const override{return b.getMainInputChannelSet()==b.getMainOutputChannelSet()&&(b.getMainOutputChannelSet()==AudioChannelSet::mono()||b.getMainOutputChannelSet()==AudioChannelSet::stereo());}
 void prepareToPlay(double rate,int)override{sr=rate;for(auto& s:st)s=State{};}
 void releaseResources()override{}
 void processBlock(AudioBuffer<float>& b,MidiBuffer&)override{
  ScopedNoDenormals nd; int nc=b.getNumChannels();if(p("in")<.5f){meter=0;return;}
  const int mode=jlimit(0,3,int(std::round(p("mode"))));
  const int detMode=mode==0?jlimit(0,2,int(std::round(p("detector")))):1;
  const int autoMode=mode==0?jlimit(0,3,int(std::round(p("auto_release")))):0;
  const double wet=jlimit(0.,1.,double(p("mix"))*.01);
  const double digitalOut=std::pow(10.,double(p("digital_out"))/20.);
  const double peakDecay=std::exp(-1./(.001*sr)), optoCharge=std::exp(-1./(.8*sr)), optoDischarge=std::exp(-1./(2.*sr));
  const double busCharge=std::exp(-1./(.6*sr)), busDischarge=std::exp(-1./(1.2*sr));
  const double optoFast=std::exp(-std::log(2.)/(.060*sr)), busFast=std::exp(-1./(.100*sr)), busSlow=std::exp(-1./(.600*sr));
  float ratio=p("ratio");if(ratio>=29.9f)ratio=1000;
  if(mode==1)ratio=20;else if(mode==2)ratio=8;else if(mode==3)ratio=30;
  float threshold=p("threshold")-22, makeup=p("out")<=-59.9f?0:Decibels::decibelsToGain(p("out"));
  bool fb=mode==0?p("type")>.5f:false,soft=mode==0?p("knee")>.5f:mode==3; constexpr bool linked=true;
  const int thrust=jlimit(0,3,int(std::round(p("thrust"))));
  const bool variable=p("variable_ar")>.5f;
  const double attackMs=mode==1?.020:mode==2?8.:mode==3?.2:variable?p("var_attack"):p("attack");
  const double releaseSec=mode==1?.050:mode==2?.100:mode==3?.300:variable?p("var_release"):p("release");
  float attack=std::exp(-1000./(attackMs*sr)),release=std::exp(-1./(releaseSec*sr));
  float rms=std::exp(-1.f/(.003f*sr));
  // Detector-only second order Butterworth high-pass; unity audio path.
  const double fc[]={0.,80.,120.,250.};
  const double w0=2*pi*std::min(fc[thrust],sr*.45)/sr, co=std::cos(w0), alpha=std::sin(w0)/std::sqrt(2.), norm=1./(1.+alpha);
  const double b0=(1.+co)*.5*norm,b1=-(1.+co)*norm,b2=b0,a1=-2.*co*norm,a2=(1.-alpha)*norm;
  float maxgr=0,maximum=0; bool colored=p("colour")>.5f;const bool nasty=p("nasty")>.5f; const double ampSmooth=std::exp(-1/(.02*sr));
  for(int i=0;i<b.getNumSamples();++i){
   float target[2]{};
   for(int ch=0;ch<nc;++ch){
    auto& s=st[ch];float x=b.getSample(ch,i),d=fb?s.feedback:x;
    if(s.hpMode!=thrust){s.hpZ1=s.hpZ2=0.;s.hpMode=thrust;}
    if(thrust!=0){const double y=b0*d+s.hpZ1;s.hpZ1=b1*d-a1*y+s.hpZ2;s.hpZ2=b2*d-a2*y;d=float(y);}
    // Track period from fractional positive crossings. Smooth the period, not instantaneous level.
    ++s.clock;
    if(x>=0 && s.old<0){
     double crossing=s.clock-x/std::max(1e-20,double(x)-s.old);
     if(s.lastCross>=0){double candidate=crossing-s.lastCross;if(candidate>=sr/20000 && candidate<=sr/20)s.period=.98*s.period+.02*candidate;}
     else s.period=sr/1000;
     s.lastCross=crossing;
    }
    s.old=x;
    double f=sr/s.period;
    // A three-sample sinusoidal predictor refines high-frequency period estimates.
    // Only accept estimates near the crossing-based value.
    s.frequency=f;
    for(double anchor:{100.,1000.,5000.,10000.})if(std::abs(f/anchor-1)<.012)s.frequency=anchor;
    double w=2*double(pi)*std::min(.499,std::abs(std::remainder(2*s.frequency,sr))/sr);
    // Squared audio has a component at twice the fundamental. Reject it before RMS smoothing.
    s.cosw=std::cos(w);s.pole=std::exp(-std::max(15.,s.frequency*.35)*2*double(pi)/sr);
    s.inputPower=s.amplitude.run(double(x)*x,s.cosw,s.pole,ampSmooth);
    const double rmsPower=mode==0?2.*s.detector.run(double(d)*d,s.cosw,s.pole,rms):double(d)*d;
    s.peakEnv=std::max(std::abs(double(d)),s.peakEnv*peakDecay);
    const double peakPower=s.peakEnv*s.peakEnv;
    // Preserve the original standard and special-mode detector laws exactly.
    double env=mode!=0?double(d)*d:detMode==0?rmsPower:detMode==1?peakPower:.5*(rmsPower+peakPower);
    float over=mode==0&&detMode==0?float(10*std::log10(std::max(1e-15,rmsPower*.5))+3.01029995664-threshold):float(10*std::log10(std::max(1e-15,env))-threshold);
    float knee=6, v=soft?(over<=-knee/2?0:over>=knee/2?over:(over+knee/2)*(over+knee/2)/(2*knee)):std::max(0.f,over);
    target[ch]=std::min(60.f,v*(fb?std::min(20.f,ratio-1):1-1/ratio));
   }
   if(linked&&nc==2)target[0]=target[1]=std::max(target[0],target[1]);
   for(int ch=0;ch<nc;++ch){
    auto& s=st[ch];float x=b.getSample(ch,i);const double t=target[ch];
    if(autoMode!=s.lastAuto){s.optoMemory=s.busMemory=0;s.lastAuto=autoMode;}
    // Independent leaky storage states represent slow optical memory and bus capacitor charge.
    // Charge with compression demand; discharge between events. Both remain bounded.
    const double demand=jlimit(0.,1.,t/12.);
    const double oc=demand>s.optoMemory?optoCharge:optoDischarge;
    const double bc=demand>s.busMemory?busCharge:busDischarge;
    s.optoMemory=oc*s.optoMemory+(1.-oc)*demand;
    s.busMemory=bc*s.busMemory+(1.-bc)*demand;
    if(t>=s.gr)s.gr=attack*s.gr+(1.-attack)*t;
    else if(autoMode==1)s.gr=std::max(t,s.gr-120./sr);
    else if(autoMode==2){
     const double slow=std::exp(-1./((.5+4.5*s.optoMemory)*sr));
     const double share=jlimit(0.,.85,s.optoMemory*.85);
     s.gr=t+(s.gr-t)*((1.-share)*optoFast+share*slow);
    }else if(autoMode==3){
     s.gr=t+(s.gr-t)*((1.-s.busMemory)*busFast+s.busMemory*busSlow);
    }else s.gr=release*s.gr+(1.-release)*t;
    double gain=std::pow(10.,-s.gr/20),y=x*gain;s.feedback=y;y*=makeup;
    double amplitude=std::sqrt(2*s.inputPower);
    if(++s.update>=128){s.update=0;
     // At unity output use original measured input level. Makeup changes drive into coloration.
     double level=20*std::log10(std::max(amplitude,1e-12))+22+p("out");
     s.c=colour.coeff(float(s.frequency),float(level),ratio,sr);}
    if((colored||nasty) && amplitude>1e-8 && makeup>0){
     double z=jlimit(-1.,1.,double(x)/amplitude),t0=1,t1=z,extra=0;
     // NASTY is a separate HF-weighted nonlinear colour stage, not extra gain reduction.
     const double freq=s.frequency;
     const double band=nasty?jlimit(0.,1.,std::log2(std::max(1000.,freq)/1000.)/std::log2(5.)):0.;
     const double top=nasty?jlimit(0.,1.,std::log2(std::max(5000.,freq)/5000.)):0.;
     const double drive=nasty?20.*std::log10(std::max(amplitude,1e-12))+22.+p("out"):0.;
     const double strength=nasty?jlimit(0.,1.,(drive+30.)/48.):0.;
     const double k2Target=band*band*(.005+.005*top);
     for(int k=2;k<=20;++k){double t=2*z*t1-t0;t0=t1;t1=t;
      if(k*freq>=.499*sr)continue;
      double coeff=colored?s.c[k-2]:0.;
      if(nasty && band>0.){
       if(k==2)coeff+=(std::copysign(k2Target,coeff==0.?1.:coeff)-coeff)*strength;
       else {const double weights[]={0.,0.,0.,.64,.40,.26,.18,.13,.09,.065,.047,.034,.025,.018,.013,.009,.007,.005,.004,.003,.002};
        coeff+=strength*k2Target*weights[k]*(k%4==0||k%4==1?-1.:1.);}
      }
      extra+=coeff*(t-(k%2==0?(k%4==0?1.:-1.):0.));}
     if(colored && mode==2 && 3*s.frequency<.499*sr){
      // Extra third harmonic peaked at 200 Hz, one octave half-power in amplitude.
      const double oct=std::log2(std::max(20.,s.frequency)/200.);
      const double weight=std::exp(-.69314718056*oct*oct);
      extra+=.01*weight*(4*z*z*z-3*z);
     }
     y+=extra*amplitude*gain*makeup;
    }
    if(wet<1.)y=wet*y+(1.-wet)*double(x);
    y*=digitalOut;
    b.setSample(ch,i,y);maxgr=std::max(maxgr,float(s.gr));maximum=std::max(maximum,float(std::abs(y)));
   }
  }
  meter=maxgr;vu=Decibels::gainToDecibels(maximum)+18;ov=maximum>=1;
 }
 void getStateInformation(MemoryBlock& dest)override{auto x=params.copyState().createXml();copyXmlToBinary(*x,dest);}
 void setStateInformation(const void* d,int n)override{if(auto x=getXmlFromBinary(d,n))params.replaceState(ValueTree::fromXml(*x));}
 bool hasEditor()const override{return true;}AudioProcessorEditor* createEditor()override;
};
class Panel:public AudioProcessorEditor,private Timer {
 Proc& p;
 static constexpr int W=252,H=896;
 static constexpr uint32 blue=0xff35b4e6, white=0xfff1f1ec;
 struct Dial:Slider {
  juce::Colour cap; bool outer;
  Dial(juce::Colour c,bool o=false):cap(c),outer(o){setSliderStyle(RotaryVerticalDrag);setTextBoxStyle(NoTextBox,false,0,0);}
  void paint(Graphics& g)override {
   auto r=getLocalBounds().toFloat().reduced(4);auto c=r.getCentre();float rad=r.getWidth()*.35f;
   double a=juce::MathConstants<double>::pi*(.75+1.5*valueToProportionOfLength(getValue()));
   auto pt=[&](double a0,float rr){return Point<float>(c.x+float(std::cos(a0))*rr,c.y+float(std::sin(a0))*rr);};
   // Machined aluminium pointer: round skirt, projecting tapered index and recessed colour cap.
   rad=r.getWidth()*.36f;
   Path body;body.addEllipse(c.x-rad,c.y-rad,2*rad,2*rad);
   Path pointer;pointer.startNewSubPath(pt(a,rad*1.52f));pointer.quadraticTo(pt(a+.19,rad*1.46f),pt(a+.34,rad*.86f));pointer.lineTo(pt(a-.34,rad*.86f));pointer.quadraticTo(pt(a-.19,rad*1.46f),pt(a,rad*1.52f));pointer.closeSubPath();body.addPath(pointer);body.setUsingNonZeroWinding(true);
   g.setColour(Colours::black.withAlpha(.8f));g.fillPath(body,AffineTransform::translation(1.5f,4));
   auto disk=Rectangle<float>(2*rad,2*rad).withCentre(c);
   g.setGradientFill(ColourGradient(Colour(0xffeeeeec),c.x-rad,c.y-rad,Colour(0xff737777),c.x+rad,c.y+rad,false));g.fillPath(body);
   g.setColour(Colour(0xffc8cbca));g.strokePath(body,PathStrokeType(.7f));
   auto inset=disk.reduced(rad*.20f);
   g.setColour(Colour(0xff535859));g.fillEllipse(inset.expanded(1));
   g.setColour(Colour(0xfff4f4ed));g.drawEllipse(inset.expanded(.5f),1.3f);
   g.setGradientFill(ColourGradient(cap.brighter(.13f),inset.getX(),inset.getY(),cap.darker(.18f),inset.getRight(),inset.getBottom(),false));g.fillEllipse(inset.reduced(1));
   if(outer){g.setColour(Colour(0xffc7cccb));g.drawEllipse(disk.reduced(2),2);}

  }
 };
 struct Toggle:ToggleButton {
  bool lamp=false,invert=false;
  void paintButton(Graphics& g,bool hover,bool down)override{
   auto r=getLocalBounds().toFloat().reduced(3);auto c=r.getCentre();bool on=getToggleState();
   g.setColour(Colours::black);g.fillEllipse(r);
   if(lamp){g.setGradientFill(ColourGradient(on?juce::Colour(0xfffff4bb):juce::Colour(0xffc2bfb1),r.getX(),r.getY(),on?juce::Colour(0xffb8a869):juce::Colour(0xff777870),r.getRight(),r.getBottom(),false));g.fillEllipse(r.reduced(2));return;}
   g.setGradientFill(ColourGradient(juce::Colour(0xffe3decc),r.getX(),r.getY(),juce::Colour(0xff55544e),r.getRight(),r.getBottom(),false));g.fillEllipse(r.reduced(1));g.setColour(juce::Colour(0xff22221f));g.fillEllipse(r.reduced(4));
   float y=c.y+((on!=invert)?-5.f:5.f);
   g.setColour(Colours::black.withAlpha(.8f));g.drawLine(c.x+2,c.y+2,c.x+2,y+2,7);
   g.setColour(juce::Colour(0xffb2b0a4));g.drawLine(c.x,c.y,c.x,y,6);
   g.setColour(juce::Colour(0xffefe9d7));g.fillEllipse(c.x-3,y-3,6,6);
   ignoreUnused(hover,down);
  }
 };
 using SA=AudioProcessorValueTreeState::SliderAttachment;using BA=AudioProcessorValueTreeState::ButtonAttachment;
 std::array<std::unique_ptr<Dial>,5> knobs;std::array<std::unique_ptr<SA>,5> ka;
 std::array<Toggle,6> buttons;std::array<std::unique_ptr<BA>,6> ba;
 struct ThrustButton:TextButton {
  void paintButton(Graphics& g,bool hover,bool down)override{
   auto r=getLocalBounds().toFloat().reduced(1);
   g.setColour(Colour(0xff090a0a));g.fillRoundedRectangle(r,3);
   g.setGradientFill(ColourGradient(Colour(down?0xff171919:0xff414444),0,1,Colour(0xff101111),0,r.getBottom(),false));g.fillRoundedRectangle(r.reduced(2),2);
   g.setColour(Colour(0xff656868));g.drawRoundedRectangle(r.reduced(1),3,1);
   g.setColour(Colour(0xfff3f4ef));g.setFont(Font(10.f,Font::bold));g.drawFittedText(getButtonText(),getLocalBounds().reduced(2),Justification::centred,1);
   ignoreUnused(hover);
  }
 } thrustButton,modeButton,advancedButton;
 ComboBox detectorBox,releaseBox; Slider mixKnob,trimKnob; ToggleButton variableAR{"Variable A/R"},nastyButton{"Nasty"};
 std::unique_ptr<BA> nastyAttach;
 std::unique_ptr<BA> variableAttach;bool timingVariable=false;
 using CA=AudioProcessorValueTreeState::ComboBoxAttachment;
 std::unique_ptr<CA> detectorAttach,releaseAttach;std::unique_ptr<SA> mixAttach,trimAttach;
 bool expanded=false;
 void showAdvanced(bool show){expanded=show;detectorBox.setVisible(show);releaseBox.setVisible(show);mixKnob.setVisible(show);trimKnob.setVisible(show);variableAR.setVisible(show);nastyButton.setVisible(show);advancedButton.setButtonText(show?"ADVANCED  -":"ADVANCED  +");setSize(W,H+ (show?357:30));repaint();}
 void txt(Graphics& g,String s,float x,float y,float w,float h,float font,uint32 colour=white,bool bold=true){
  g.setColour(juce::Colour(colour));g.setFont(Font(font,bold?Font::bold:Font::plain));g.drawFittedText(s,Rectangle<float>(x,y,w,h).toNearestInt(),Justification::centred,1);
 }
 void screw(Graphics& g,float x,float y){Rectangle<float> r(x-12,y-12,24,24);g.setColour(Colours::black);g.fillEllipse(r);g.setColour(juce::Colour(0xff655f52));g.drawEllipse(r.reduced(2),1);g.drawLine(x-5,y-5,x+5,y+5,2);g.drawLine(x-5,y+5,x+5,y-5,2);}
 void scale(Graphics& g,float x,float y,float radius,std::initializer_list<const char*> labels,uint32 color=white){
  int i=0,n=int(labels.size());for(auto s:labels){double a=juce::MathConstants<double>::pi*(.75+1.5*i++/std::max(1,n-1));float xx=x+float(std::cos(a))*radius,yy=y+float(std::sin(a))*radius;g.setColour(juce::Colour(blue));g.fillEllipse(xx-2.5f,yy-2.5f,5,5);float rr=radius+13;txt(g,s,x+float(std::cos(a))*rr-15,y+float(std::sin(a))*rr-8,30,16,12,color);}
 }
public:
 Panel(Proc& proc):AudioProcessorEditor(proc),p(proc){
  const char* ids[]={"out","threshold","attack","release","ratio"};
  unsigned cs[]={0xff202223,0xff087b82,0xffc6d0d3,0xffebf3f5,0xffed302b};
  Rectangle<int> b[]={{125,117,82,82},{125,270,82,82},{130,430,72,72},{130,548,72,72},{126,671,80,80}};
  for(int i=0;i<5;++i){knobs[i]=std::make_unique<Dial>(juce::Colour(cs[i]),false);addAndMakeVisible(*knobs[i]);knobs[i]->setBounds(b[i]);ka[i]=std::make_unique<SA>(p.params,ids[i],*knobs[i]);knobs[i]->setTooltip(ids[i]);}
  const char* bs[]={"meter","link","type","knee","thrust","in"};int yy[]={439,507,575,643,711,747};
  for(int i=0;i<6;++i){if(i==1||i==4)continue;buttons[i].lamp=i==5;buttons[i].invert=(i==0||i==2||i==3);addAndMakeVisible(buttons[i]);buttons[i].setBounds(i==5?65:15,yy[i],i==5?42:26,i==5?42:26);ba[i]=std::make_unique<BA>(p.params,bs[i],buttons[i]);buttons[i].setTooltip(bs[i]);}
  addAndMakeVisible(thrustButton);thrustButton.setBounds(12,710,34,27);
  thrustButton.setTooltip("HPF: Off / 80 Hz / 120 Hz / 250 Hz");
  thrustButton.onClick=[this]{if(auto* param=p.params.getParameter("thrust")){int next=(int(std::round(p.p("thrust")))+1)%4;param->beginChangeGesture();param->setValueNotifyingHost(param->convertTo0to1(float(next)));param->endChangeGesture();}repaint();};
  thrustButton.setButtonText(String(int(std::round(p.p("thrust")))));
  addAndMakeVisible(modeButton);modeButton.setBounds(12,506,82,27);
  modeButton.setTooltip("MODE: Off / Detonate / Explode / Burst");
  modeButton.onClick=[this]{if(auto* param=p.params.getParameter("mode")){int next=(int(std::round(p.p("mode")))+1)%4;param->beginChangeGesture();param->setValueNotifyingHost(param->convertTo0to1(float(next)));param->endChangeGesture();}};
  addAndMakeVisible(advancedButton);advancedButton.setBounds(8,H+3,W-16,24);
  advancedButton.onClick=[this]{showAdvanced(!expanded);};
  detectorBox.addItemList(StringArray{"527 RMS","Peak","Hybrid"},1);
  releaseBox.addItemList(StringArray{"Manual","VCA - dbx","OPTO","BVCA - SSL"},1);
  for(auto* box:{&detectorBox,&releaseBox}){addChildComponent(*box);box->setColour(ComboBox::backgroundColourId,Colour(0xff151717));box->setColour(ComboBox::textColourId,Colour(white));box->setColour(ComboBox::outlineColourId,Colour(0xff666969));}
  detectorBox.setBounds(16,H+52,W-32,26);releaseBox.setBounds(16,H+100,W-32,26);
  addChildComponent(mixKnob);mixKnob.setSliderStyle(Slider::RotaryHorizontalVerticalDrag);mixKnob.setTextBoxStyle(Slider::TextBoxRight,false,54,22);mixKnob.setTextValueSuffix("%");mixKnob.setBounds(85,H+139,145,68);
  addChildComponent(trimKnob);trimKnob.setSliderStyle(Slider::RotaryHorizontalVerticalDrag);trimKnob.setTextBoxStyle(Slider::TextBoxRight,false,65,22);trimKnob.setTextValueSuffix(" dB");trimKnob.setDoubleClickReturnValue(true,0);trimKnob.setBounds(85,H+209,145,68);trimKnob.setColour(Slider::rotarySliderFillColourId,Colour(blue));trimKnob.setColour(Slider::textBoxTextColourId,Colour(white));trimKnob.setColour(Slider::textBoxOutlineColourId,Colours::transparentBlack);
  mixKnob.setColour(Slider::rotarySliderFillColourId,Colour(blue));mixKnob.setColour(Slider::textBoxTextColourId,Colour(white));mixKnob.setColour(Slider::textBoxOutlineColourId,Colours::transparentBlack);
  detectorAttach=std::make_unique<CA>(p.params,"detector",detectorBox);releaseAttach=std::make_unique<CA>(p.params,"auto_release",releaseBox);mixAttach=std::make_unique<SA>(p.params,"mix",mixKnob);trimAttach=std::make_unique<SA>(p.params,"digital_out",trimKnob);
  addChildComponent(variableAR);variableAR.setBounds(16,H+283,W-32,27);variableAR.setColour(ToggleButton::textColourId,Colour(white));variableAR.setColour(ToggleButton::tickColourId,Colour(blue));variableAR.setTooltip("Switch the front-panel timing knobs to independent continuous digital attack/release ranges");variableAttach=std::make_unique<BA>(p.params,"variable_ar",variableAR);
  addChildComponent(nastyButton);nastyButton.setBounds(16,H+313,W-32,27);nastyButton.setColour(ToggleButton::textColourId,Colour(white));nastyButton.setColour(ToggleButton::tickColourId,Colour(0xffff603d));nastyButton.setTooltip("Nasty: aggressive upper-mid and high-frequency harmonics, available in every mode");nastyAttach=std::make_unique<BA>(p.params,"nasty",nastyButton);
  showAdvanced(false);timerCallback();startTimerHz(20);
 }
 void timerCallback()override{
  const bool variable=p.p("variable_ar")>.5f;
  if(variable!=timingVariable){for(int i=2;i<=3;++i){ka[i].reset();ka[i]=std::make_unique<SA>(p.params,variable?(i==2?"var_attack":"var_release"):(i==2?"attack":"release"),*knobs[i]);}timingVariable=variable;}
  const int mode=jlimit(0,3,int(std::round(p.p("thrust"))));thrustButton.setButtonText(mode==0?"OFF":String(mode));const int fx=jlimit(0,3,int(std::round(p.p("mode"))));const char* names[]={"OFF","DETONATE","EXPLODE","BURST"};modeButton.setButtonText(names[fx]);for(int i=2;i<5;++i){const bool active=fx==0&&(i!=3||p.p("auto_release")<.5f);knobs[i]->setEnabled(active);knobs[i]->setAlpha(active?1.f:.55f);}detectorBox.setEnabled(fx==0);releaseBox.setEnabled(fx==0);variableAR.setEnabled(fx==0);buttons[2].setEnabled(fx==0);buttons[3].setEnabled(fx==0);repaint();}
 void paint(Graphics& g)override{
  g.fillAll(juce::Colour(0xff202222));
  Random rng(527);for(int i=0;i<12500;++i){g.setColour((i%2?Colours::white:Colours::black).withAlpha(.045f));g.fillRect(rng.nextFloat()*W,rng.nextFloat()*H,1.f,1.f);}
  g.setColour(juce::Colour(0xff444646));g.drawRect(getLocalBounds(),1);
  g.setColour(Colour(0xff090a0a));g.fillEllipse(110,9,32,32);g.fillEllipse(110,849,32,32);
  txt(g,"527",170,20,55,24,21,blue);
  screw(g,29,82);screw(g,223,82);screw(g,157,407);screw(g,29,802);screw(g,223,802);
  txt(g,"GR",42,98,25,13,11);txt(g,"VU",72,98,25,13,11);txt(g,"OV",100,98,25,13,11);
  g.setColour(Colour(blue));g.fillRect(26,115,3,294);
  const float gr[]={1.5,4,6,8,10,12,15,17,19,23};const int vu[]={3,2,1,0,-1,-3,-5,-7,-10,-20};
  bool v=p.p("meter")>.5f;
  for(int i=0;i<10;++i){int yy=115+i*31;txt(g,String(-gr[i],i==0?1:0),30,yy,33,18,11);txt(g,String(vu[i]),83,yy,24,18,11);g.setColour(Colours::black);g.fillEllipse(64,yy,19,19);g.setColour((v?p.vu.load()>=vu[i]:p.meter.load()>=gr[i])?Colour(0xffff4828):Colour(0xff43494a));g.fillEllipse(66,yy+2,15,15);g.setColour(Colours::white.withAlpha(.17f));g.fillEllipse(68,yy+3,5,3);}
  g.setColour(p.ov.load()>0?Colours::red:Colour(0xff43494a));g.fillEllipse(109,115,16,16);
  auto dot=[&](float xx,float yy){g.setColour(Colour(blue));g.fillEllipse(xx-2,yy-2,4,4);};
  auto mark=[&](float cx,float cy,float radius,double deg,String label,uint32 color,float fs=11.f){double a=deg*pi/180.;float xx=cx+std::cos(a)*radius,yy=cy+std::sin(a)*radius;dot(xx,yy);float rr=radius+10;txt(g,label,cx+std::cos(a)*rr-11,cy+std::sin(a)*rr-7,22,14,fs,color);};
  auto arc=[&](float xx,float yy,float radius,std::initializer_list<const char*> labels,uint32 color){int i=0,n=int(labels.size());for(auto s:labels){mark(xx,yy,radius,135+270.*i++/std::max(1,n-1),String::fromUTF8(s),color);}};
  arc(166,158,45,{"","","","","0","10"},white);
  txt(g,String::charToString(0x221e),121,211,20,13,12);txt(g,"dB",153,211,26,13,11);
  arc(166,311,45,{"10","7","4","0","-3","-6","-20"},white);txt(g,"dBu",150,362,32,14,11);
  // Dedicated rightmost twelve-pixel column; no numeric labels enter it.
  auto vertical=[&](String s,int yy){for(int i=0;i<s.length();++i)txt(g,s.substring(i,i+1),237,yy+i*13,13,13,12);};
  vertical("OUT",141);vertical("THRESH",280);
  // Independently positioned labels for concentric scales, avoiding the switch column.
  if(!timingVariable)arc(166,466,37,{"1","5","10","15","20","25"},white);else{txt(g,".02",114,490,27,13,10);txt(g,"100",193,490,27,13,10);txt(g,String(p.p("var_attack"),2),140,407,52,15,11,blue);}
  txt(g,"ATTACK (ms)",115,512,102,16,11);
  if(!timingVariable)arc(166,584,37,{".3",".5","1","1.5","2","2.5","3"},blue);else{txt(g,".01",114,608,27,13,10);txt(g,"5",193,608,27,13,10);txt(g,String(p.p("var_release"),3),140,526,52,15,11,blue);}
  txt(g,"RELEASE (s)",115,627,102,16,11,blue);
  arc(166,711,44,{"1","1.5","2.5","3","4","6","10",""},white);txt(g,String::charToString(0x221e),205,754,22,14,13);txt(g,"RATIO",142,772,49,16,12);
  const char* names[]={"METER","LINK","TYPE","KNEE","HPF"};const char* upper[]={"GR","IN","NEW","HARD","IN"};const char* lower[]={"VU","OUT","OLD","SOFT","OUT"};
  for(int i=0;i<5;++i){int yy=439+i*68;if(i==1){txt(g,"MODE",12,492,82,13,11,blue);continue;}txt(g,names[i],i==4?48:43,yy+4,65,16,12,blue);if(i!=4&&i!=1){txt(g,upper[i],9,yy-13,36,13,10);txt(g,lower[i],9,yy+25,36,12,10);}}
  const char* freqLabel[]={"","80 Hz","120 Hz","250 Hz"};txt(g,freqLabel[jlimit(0,3,int(std::round(p.p("thrust"))))],49,730,62,13,10);
  txt(g,"IN",39,760,25,16,12);
  g.setColour(Colour(0xff131515));g.fillRect(0,H,W,getHeight()-H);
  if(expanded){txt(g,"DETECTOR",16,H+34,W-32,15,11,blue);txt(g,"RELEASE CHARACTER",16,H+82,W-32,15,11,blue);txt(g,"MIX",17,H+159,60,20,12,blue);txt(g,"DIGITAL OUT",8,H+229,82,20,11,blue);}


 }
};
AudioProcessorEditor* Proc::createEditor(){return new Panel(*this);}
AudioProcessor* JUCE_CALLTYPE createPluginFilter(){return new Proc();}

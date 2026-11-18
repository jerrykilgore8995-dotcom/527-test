#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_dsp/juce_dsp.h>
#include <BinaryData.h>
#include <array>
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
 std::array<float,19> coeff(float f,float l,float ratio) const {
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
  }return out;
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
  add("ratio","Ratio",1,30,4,.4);
  for(auto id:{"meter","link","type","knee","thrust","in","colour"})l.add(std::make_unique<AudioParameterBool>(ParameterID{id,1},id,String(id)=="link"||String(id)=="in"||String(id)=="colour"));
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
  float ratio=p("ratio");if(ratio>=29.9f)ratio=1000;
  float threshold=p("threshold")-22, makeup=p("out")<=-59.9f?0:Decibels::decibelsToGain(p("out"));
  bool fb=p("type")>.5f,soft=p("knee")>.5f,linked=p("link")>.5f,thrust=p("thrust")>.5f;
  float attack=std::exp(-1000.f/(p("attack")*sr)),release=std::exp(-1.f/(p("release")*sr));
  float rms=std::exp(-1.f/(.003f*sr)),hp=std::exp(-2*pi*150/sr);
  float maxgr=0,maximum=0; bool colored=p("colour")>.5f; const double ampSmooth=std::exp(-1/(.02*sr));
  for(int i=0;i<b.getNumSamples();++i){
   float target[2]{};
   for(int ch=0;ch<nc;++ch){
    auto& s=st[ch];float x=b.getSample(ch,i),d=fb?s.feedback:x;
    s.lp=hp*s.lp+(1-hp)*d;if(thrust)d-=.8f*s.lp;
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
    double env=s.detector.run(double(d)*d,s.cosw,s.pole,rms);
    float over=float(10*std::log10(std::max(1e-15,env))+3.01029995664-threshold);
    float knee=6, v=soft?(over<=-knee/2?0:over>=knee/2?over:(over+knee/2)*(over+knee/2)/(2*knee)):std::max(0.f,over);
    target[ch]=std::min(60.f,v*(fb?std::min(20.f,ratio-1):1-1/ratio));
   }
   if(linked&&nc==2)target[0]=target[1]=std::max(target[0],target[1]);
   for(int ch=0;ch<nc;++ch){
    auto& s=st[ch];float x=b.getSample(ch,i),a=target[ch]>s.gr?attack:release;s.gr=a*s.gr+(1-a)*target[ch];
    double gain=std::pow(10.,-s.gr/20),y=x*gain;s.feedback=y;y*=makeup;
    double amplitude=std::sqrt(2*s.inputPower);
    if(++s.update>=128){s.update=0;
     // At unity output use original measured input level. Makeup changes drive into coloration.
     double level=20*std::log10(std::max(amplitude,1e-12))+22+p("out");
     s.c=colour.coeff(float(s.frequency),float(level),ratio);}
    if(colored && amplitude>1e-8 && makeup>0){
     double z=jlimit(-1.,1.,double(x)/amplitude),t0=1,t1=z,extra=0;
     for(int k=2;k<=20;++k){double t=2*z*t1-t0;t0=t1;t1=t;
      if(k*s.frequency<.499*sr)extra+=s.c[k-2]*(t-(k%2==0?(k%4==0?1.:-1.):0.));}
     y+=extra*amplitude*gain*makeup;
    }
    b.setSample(ch,i,y);maxgr=std::max(maxgr,float(s.gr));maximum=std::max(maximum,float(std::abs(y)));
   }
  }
  meter=maxgr;vu=Decibels::gainToDecibels(maximum)+18;ov=maximum>=1;
 }
 void getStateInformation(MemoryBlock& dest)override{auto x=params.copyState().createXml();copyXmlToBinary(*x,dest);}
 void setStateInformation(const void* d,int n)override{if(auto x=getXmlFromBinary(d,n))params.replaceState(ValueTree::fromXml(*x));}
 bool hasEditor()const override{return true;}AudioProcessorEditor* createEditor()override;
};
class Panel:public AudioProcessorEditor,private Timer{
 Proc& p;
 struct Dial:Slider {juce::Colour face;Dial(juce::Colour c):face(c){setSliderStyle(RotaryVerticalDrag);setTextBoxStyle(NoTextBox,false,0,0);}
 void paint(Graphics& g)override{auto r=getLocalBounds().toFloat().reduced(2);float a=pi*(.75f+1.5f*valueToProportionOfLength(getValue()));g.setColour(Colours::black);g.fillEllipse(r);g.setColour(juce::Colour(0xffd0cbbc));g.fillEllipse(r.reduced(2));g.setColour(face);g.fillEllipse(r.reduced(r.getWidth()*.19f));auto c=r.getCentre();g.setColour(Colours::white);g.drawLine(c.x+std::cos(a)*r.getWidth()*.31f,c.y+std::sin(a)*r.getWidth()*.31f,c.x+std::cos(a)*r.getWidth()*.48f,c.y+std::sin(a)*r.getWidth()*.48f,2);}};
 using SA=AudioProcessorValueTreeState::SliderAttachment;using BA=AudioProcessorValueTreeState::ButtonAttachment;
 std::array<std::unique_ptr<Dial>,5> knobs;std::array<std::unique_ptr<SA>,5> ka;
 std::array<ToggleButton,6> buttons;std::array<std::unique_ptr<BA>,6> ba;
public:
 Panel(Proc& proc):AudioProcessorEditor(proc),p(proc){
  const char* ids[]={"out","threshold","attack","release","ratio"};
  unsigned cs[]={0xff343633,0xff378c89,0xffb8b5a7,0xffede6d4,0xffc32f20};
  Rectangle<int> bounds[]={{144,88,71,71},{144,222,71,71},{130,373,94,94},{153,396,48,48},{135,553,90,90}};
  for(int i=0;i<5;++i){knobs[i]=std::make_unique<Dial>(juce::Colour(cs[i]));addAndMakeVisible(*knobs[i]);knobs[i]->setBounds(bounds[i]);ka[i]=std::make_unique<SA>(p.params,ids[i],*knobs[i]);knobs[i]->setTooltip(ids[i]);}
  const char* bs[]={"meter","link","type","knee","thrust","in"};int yy[]={350,407,464,521,578,672};
  for(int i=0;i<6;++i){addAndMakeVisible(buttons[i]);buttons[i].setBounds(20,yy[i],26,26);ba[i]=std::make_unique<BA>(p.params,bs[i],buttons[i]);buttons[i].setTooltip(bs[i]);}
  setSize(260,760);startTimerHz(20);
 }
 void timerCallback()override{repaint();}
 void paint(Graphics& g)override{
  g.fillAll(juce::Colour(0xff282621));g.setColour(juce::Colour(0xff48a3c4));g.setFont(20);g.drawText("527",172,20,70,28,Justification::centred);
  auto label=[&](String t,int x,int y,int w=70){g.setFont(13);g.drawText(t,x,y,w,18,Justification::centred);};
  g.setColour(Colours::whitesmoke);label("GR VU OV",12,78,105);
  const float gr[]={1.5,4,6,8,10,12,15,17,19,23};const int vu[]={3,2,1,0,-1,-3,-5,-7,-10,-20};
  bool v=p.p("meter")>.5f;
  for(int i=0;i<10;++i){int y=104+i*22;g.setColour(Colours::whitesmoke);label(String(-gr[i],gr[i]==1.5f?1:0),8,y,42);label(String(vu[i]),66,y,32);g.setColour((v?p.vu.load()>=vu[i]:p.meter.load()<gr[i])?juce::Colour(0xffff4500):juce::Colour(0xff49362b));g.fillEllipse(51,y+3,13,13);}
  g.setColour(p.ov.load()>0?Colours::red:Colours::darkred);g.fillEllipse(95,100,9,9);
  g.setColour(juce::Colour(0xff48a3c4));label("OUT",181,165);label("THRESH",181,297);label("ATTACK",102,473);label("RELEASE",183,473);label("RATIO",142,650);
  const char* names[]={"METER","LINK","TYPE","KNEE","THRUST","IN"};for(int i=0;i<6;++i)label(names[i],46,(i==5?672:350+i*57),65);
  g.setColour(Colours::whitesmoke);label("GR / VU",8,330,85);label("IN",8,388,37);label("NEW / OLD",5,444,96);label("HARD / SOFT",5,501,102);label("IN",8,559,37);
  label("-inf    dB    10",135,182,110);label("+10   dBu   -20",129,314,120);label("1–25 ms",110,492,68);label(".3–3 s",182,492,68);label("1    3    6    10    ∞",113,632,138);
  for(auto y:{53,711})for(auto x:{16,234}){g.setColour(Colours::black);g.fillEllipse(x,y,12,12);g.setColour(Colours::grey);g.drawLine(x+3,y+6,x+9,y+6,1);}
  g.setColour(Colours::black);g.fillEllipse(122,8,16,16);g.fillEllipse(122,736,16,16);
 }
};
AudioProcessorEditor* Proc::createEditor(){return new Panel(*this);}
AudioProcessor* JUCE_CALLTYPE createPluginFilter(){return new Proc();}

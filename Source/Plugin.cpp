#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_dsp/juce_dsp.h>
#include <BinaryData.h>
#include <array>
#include <cmath>
#include <atomic>
using namespace juce;
constexpr float pi=3.14159265358979323846f;
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
    r.a[k-2]=amp*std::cos(phase);r.b[k-2]=amp*std::sin(phase);
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
struct State {float env=0,gr=0,feedback=0,lp=0,peak=0,old=0,frequency=1000;int crossing=0,update=0;std::array<float,19> c{};};
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
  for(auto id:{"meter","link","type","knee","thrust","in"})l.add(std::make_unique<AudioParameterBool>(ParameterID{id,1},id,String(id)=="link"||String(id)=="in"));
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
  float maxgr=0,maximum=0;
  for(int i=0;i<b.getNumSamples();++i){
   float target[2]{};
   for(int ch=0;ch<nc;++ch){
    auto& s=st[ch];float x=b.getSample(ch,i),d=fb?s.feedback:x;
    s.lp=hp*s.lp+(1-hp)*d;if(thrust)d-=.8f*s.lp;
    s.env=rms*s.env+(1-rms)*d*d;
    float over=10*std::log10(std::max(1e-15f,s.env))+3.0103f-threshold;
    float knee=6, v=soft?(over<=-knee/2?0:over>=knee/2?over:(over+knee/2)*(over+knee/2)/(2*knee)):std::max(0.f,over);
    target[ch]=std::min(60.f,v*(fb?std::min(20.f,ratio-1):1-1/ratio));
   }
   if(linked&&nc==2)target[0]=target[1]=std::max(target[0],target[1]);
   for(int ch=0;ch<nc;++ch){
    auto& s=st[ch];float x=b.getSample(ch,i),a=target[ch]>s.gr?attack:release;s.gr=a*s.gr+(1-a)*target[ch];
    float y=x*Decibels::decibelsToGain(-s.gr);s.feedback=y;y*=makeup;
    // Track input frequency and peak; bounded single-tone coloration approximation.
    ++s.crossing;if(x>=0&&s.old<0&&s.crossing>2){s.frequency=.8f*s.frequency+.2f*float(sr/s.crossing);s.crossing=0;}s.old=x;
    s.peak=std::max(std::abs(y),s.peak*float(std::exp(-1/(.05*sr))));
    if(++s.update>=128){s.update=0;s.c=colour.coeff(s.frequency,Decibels::gainToDecibels(std::abs(x)+1e-9f)+22,ratio);}
    float z=jlimit(-1.f,1.f,y/std::max(s.peak,1e-6f)),t0=1,t1=z,extra=0;
    for(int k=2;k<=20;++k){float t=2*z*t1-t0;t0=t1;t1=t;
     if(k*s.frequency<.43*sr)extra+=s.c[k-2]*(t-(k%2==0?std::cos(k*pi/2):0));}
    y+=extra*s.peak;
    b.setSample(ch,i,y);maxgr=std::max(maxgr,s.gr);maximum=std::max(maximum,std::abs(y));
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

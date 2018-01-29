#ifndef __LOWPASS2_H__
#define __LOWPASS2_H__

// 2nd order IIR low pass (averaging) filter
// When IntegScale2=IntegScale1-2 => no overshoot
// When IntegScale2=IntegScale1-1 => overshoots by about 4%

template <class Int=int64_t, int IntegScale1=10, int IntegScale2=8, int InpScale=16>
class LowPass2
{ public:
   // static const int InpScale    = 16;
   // static const int IntegScale1 = 10;
   // static const int IntegScale2 = IntegScale-2;
   Int Out1, Out;
  public:
   void Set(Int Inp=0) { Out1=Out = Inp<<InpScale; }
   Int  Process( Int Inp)
   { Inp<<=InpScale;
     Out1 = ( Inp -Out + (Out1<<IntegScale1) + (1<<(IntegScale1-1)) )>>IntegScale1;
     Out  = ( Out1-Out + (Out <<IntegScale2) + (1<<(IntegScale2-1)) )>>IntegScale2;
     return  Out ; }   // return fractional result
   Int getOutput(void)
   { return ( Out + (1<<(InpScale-1)) )>>InpScale; }
} ;


// make filter with gradually growing integration factors.

#endif // __LOWPASS2_H__

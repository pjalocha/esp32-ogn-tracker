#include <stdint.h>

template <class Int>
 class SlopePipe
{ public:
   Int Data[4];
   uint8_t Ptr;

   Int Aver, Slope; // [1/ 4 Inp  ] Average and Slope estimate
   Int Noise;       // [1/16 Inp^2] Average squared residue

  public:
   void Clear(Int Inp=0)
   { for(Ptr=0; Ptr<4; Ptr++)
       Data[Ptr]=Inp;
     Ptr=0; }

   void Input(Int Inp)
   { Data[Ptr++]=Inp; Ptr&=3; }

   void FitSlope(void)
   { Int A =Data[Ptr++]; Ptr&=3;
     Int B =Data[Ptr++]; Ptr&=3;
     Int C =Data[Ptr++]; Ptr&=3;
     Int D =Data[Ptr++]; Ptr&=3;
     Aver = A+B+C+D;
     Slope = (C+D)-(A+B)+((D-A)<<1);
     Slope = ((Slope<<1))/5;
   }

   void CalcNoise(void)
   { Int Extr, Diff;
     Extr = Aver-Slope-(Slope>>1);
     Noise=0;
     for(uint8_t Idx=0; Idx<4; Idx++)
     { Diff = (Data[Ptr++]<<2)-Extr; Ptr&=3;
       Noise+=Diff*Diff;
       Extr+= Slope; }
     Noise=(Noise+2)>>2;
   }

} ;



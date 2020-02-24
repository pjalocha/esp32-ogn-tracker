#include <stdint.h>
#include "st7789.h"

class LCD_BattSymb
{ public:
   static const uint8_t Width = 24;
   static const uint8_t Border = 2;
   static const uint8_t Cells = 5;
   static const uint8_t CellWidth  = Width-Border*4;
   static const uint8_t CellLength = CellWidth*3/4;
   static const uint8_t Length = (CellLength+Border)*Cells+Border*3;
   static const uint8_t TipLen = Border*2;
   int16_t Xpos, Ypos;
   int16_t FrameCol, CellCol, FillCol;
   uint8_t CellMap;                                                       // which cells are to be displayed
   uint8_t Flags;                                                         // which cells and other elements are displayed

  public:
   LCD_BattSymb() { CellMap=0; Xpos=0; Ypos=0; FillCol=RGB565_LIGHTGREY; CellCol=RGB565_DARKGREEN; FrameCol=RGB565_BLACK; Flags=0; }

   void setLevel(uint8_t Level)
   { CellMap=0; uint8_t Mask=1;
     if(Level>Cells) Level=Cells;
     for(uint8_t Cell=0; Cell<Level; Cell++)
     { CellMap|=Mask; Mask<<=1; }                                                        // set cells to be displayed after the battery level
   }

   void Draw(void)
   { LCD_DrawBox(Xpos, Ypos, Length, Width, FrameCol);                                   // frame box
     LCD_DrawBox(Xpos+Length, Ypos+Width/4, TipLen, Width/2, FrameCol);                  // tip
     LCD_DrawBox(Xpos+Border, Ypos+Border, Length-Border*2, Width-Border*2, FillCol);    // inner space
     uint8_t Mask=1;
     for(uint8_t Cell=0; Cell<Cells; Cell++)
     { if(CellMap&Mask)
       { uint16_t Xofs=Border*2+(CellLength+Border)*Cell;
         LCD_DrawBox(Xpos+Xofs, Ypos+Border*2, CellLength, CellWidth, CellCol); }
       Mask<<=1; }
     Flags = 0x80 | CellMap; }

   void Update(void)
   { uint8_t Mask=1;
     for(uint8_t Cell=0; Cell<Cells; Cell++)
     { if((CellMap^Flags)&Mask)
       { uint16_t Xofs=Border*2+(CellLength+Border)*Cell;
         LCD_DrawBox(Xpos+Xofs, Ypos+Border*2, CellLength, CellWidth, CellMap&Mask ? CellCol:FillCol); } // display or erase a cell
       Mask<<=1; }
     Flags = 0x80 | CellMap; }

} ;


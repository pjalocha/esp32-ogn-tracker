#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <vector>
#include <algorithm>

#include "format.h"
#include "ognconv.h"

static bool Earlier(const char *Line1, const char *Line2) { return strcmp(Line1, Line2)<0; }

static int Verbose    =  0;
static int GeoidSepar = 40;

const int MaxLineLen = 256;
const int MaxIGClen  =  64;

static FILE *OutFile = 0;

int main(int argc, char *argv[])
{ if(argc<2)
  { printf("Usage: %s <own-aircraft-APRS-call> <input-file.aprs>\n", argv[0]);
    return 0; }

  const char *OwnAcft = argv[1]; int OwnAcftLen = strlen(OwnAcft);       // target aircraft APRS name
  char OutFileName[32]; strcpy(OutFileName, OwnAcft); strcat(OutFileName, ".IGC"); // create the IGC file name

  const char *InpFileName = argv[2];
  FILE *InpFile;
  if(InpFileName==0 || strcmp(InpFileName,"-")==0) InpFile=stdin;
  else
  { InpFile=fopen(InpFileName, "rt");
    if(InpFile==0) { printf("Cannot open %s for read\n", InpFileName); return 0; }
  }

  std::vector<char *> OutLine;                                           // list of APRS lines for the selected aircraft
  char InpLine[MaxLineLen];
  // char OutLine[MaxLineLen];
  int InpLines=0;
  // int OutLines=0;
  char *Out=0;                                                           // (new) IGC line allocated
  for( ; ; )
  { if(fgets(InpLine, MaxLineLen, InpFile)==0) break;                    // read line from the input
    char *EOL = strchr(InpLine, '\n'); if(EOL==0) break;
    *EOL = 0;
    InpLines++;
    if(memcmp(InpLine, OwnAcft, OwnAcftLen)) continue;                   // must start with the selected APRS name
    if(Out==0) Out = (char *)malloc(MaxIGClen+MaxLineLen);               // allocated (new) IGC line
    int OutLen=APRS2IGC(Out, InpLine, GeoidSepar);                       // convert APRS to IGC B-record
    if(OutLen>0)                                                         // if correct conversion
    { // if(Verbose) printf("%s => %s [%d]\n", InpLine, Out, OutLen);
      OutLen += Format_String(Out+OutLen, "LGNE ");
      OutLen += Format_String(Out+OutLen, InpLine);
      Out[OutLen++] = '\n'; Out[OutLen] = 0;
      OutLine.push_back(Out); Out=0; }                                   // add the new IGC line to the list
  }
  if(Out) { free(Out); Out=0; }
  if(InpFile!=stdin) fclose(InpFile);
  printf("%d lines from %s\n", InpLines, InpFileName);

  std::sort(OutLine.begin(), OutLine.end(), Earlier);                    // sort the IGC B-record lines
  OutFile=fopen(OutFileName, "wt");
  if(OutFile==0) { printf("Cannot open %s for write\n", OutFileName); return 0; }
  for(size_t Idx=0; Idx<OutLine.size(); Idx++)                           // look over (now sorted) IGC B-records
  { fprintf(OutFile, "%s", OutLine[Idx]); }
  fclose(OutFile);
  printf("%lu lines to %s\n", OutLine.size(), OutFileName);

  return 0; }

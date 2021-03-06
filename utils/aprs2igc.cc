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

static int Verbose    =  1;
static int GeoidSepar = 40;

static FILE *OutFile = 0;

int main(int argc, char *argv[])
{ if(argc<2)
  { printf("Usage: %s <own-aircraft-APRS-call> <input-file.aprs>\n", argv[0]);
    return 0; }

  const char *OwnAcft = argv[1]; int OwnAcftLen = strlen(OwnAcft);
  char OutFileName[32]; strcpy(OutFileName, OwnAcft); strcat(OutFileName, ".IGC");

  const char *InpFileName = argv[2];
  FILE *InpFile;
  if(InpFileName==0 || strcmp(InpFileName,"-")==0) InpFile=stdin;
  else
  { InpFile=fopen(InpFileName, "rt");
    if(InpFile==0) { printf("Cannot open %s for read\n", InpFileName); return 0; }
  }

  std::vector<char *> OutLine;
  char InpLine[256];
  // char OutLine[256];
  int InpLines=0;
  // int OutLines=0;
  char *Out=0;
  for( ; ; )
  { if(fgets(InpLine, 256, InpFile)==0) break;
    char *EOL = strchr(InpLine, '\n'); if(EOL==0) break;
    *EOL = 0;
    InpLines++;
    if(memcmp(InpLine, OwnAcft, OwnAcftLen)) continue;
    if(Out==0) Out = (char *)malloc(60);
    int OutLen=APRS2IGC(Out, InpLine, GeoidSepar);
    if(OutLen>0)
    { if(Verbose) printf("%s => %s [%d]\n", InpLine, Out, OutLen);
      OutLine.push_back(Out); Out=0; }
  }
  if(Out) { free(Out); Out=0; }
  if(InpFile!=stdin) fclose(InpFile);
  printf("%d lines from %s\n", InpLines, InpFileName);

  std::sort(OutLine.begin(), OutLine.end(), Earlier);
  OutFile=fopen(OutFileName, "wt");
  if(OutFile==0) { printf("Cannot open %s for write\n", OutFileName); return 0; }
  for(size_t Idx=0; Idx<OutLine.size(); Idx++)
  { fprintf(OutFile, "%s", OutLine[Idx]); }
  fclose(OutFile);
  printf("%lu lines to %s\n", OutLine.size(), OutFileName);

  return 0; }

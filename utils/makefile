read_log:	read_log.cc
	g++ -Wall -Wno-misleading-indentation -O2 -o read_log read_log.cc format.cpp

tlg2aprs:	tlg2aprs.cc
	g++ -Wall -Wno-misleading-indentation -O2 -o tlg2aprs tlg2aprs.cc format.cpp ognconv.cpp

aprs2igc:	aprs2igc.cc
	g++ -Wall -Wno-misleading-indentation -O2 -o aprs2igc aprs2igc.cc format.cpp ognconv.cpp

aprs2igc.exe:	aprs2igc.cc
	x86_64-w64-mingw32-g++ -Wall -Wno-misleading-indentation -O2 -static -o aprs2igc.exe aprs2igc.cc format.cpp

serial_dump:	serial_dump.cc
	g++ -Wall -Wno-misleading-indentation -O2 -o serial_dump serial_dump.cc format.cpp

clean:
	rm read_log aprs2igc serial_dump


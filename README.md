# Teensy-QCX 
There are 3 separate versions here.\
\
The baseband version uses a Hilbert filter covering +-20 khz, decodes USB or LSB in an adder, and runs a final filter for bandwidth.\
\
The IF version also uses a wide band Hilbert.  A bandpass filter picks out a slice at an IF around 8k - 9k audio and a bfo is used \
to translate to baseband.\
\
The Weaver version receives at baseband using the Weaver method.  There is a wide band Hilbert for the waterfall display.\
\
Each version is at a different level of completion.


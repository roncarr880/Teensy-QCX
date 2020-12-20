# Teensy-QCX 
There are 3 separate versions here.\
\
The baseband version uses a Hilbert filter covering +-20 khz, decodes USB or LSB in an adder, and runs a final filter for bandwidth.\
\
The IF version also uses a wide band Hilbert.  A bandpass filter picks out a slice at an IF around 8k - 9k audio and a bfo is used 
to translate to baseband. I have decided to add to this version extra features.\
\
The Weaver version receives at baseband using the Weaver method.  There is a wide band Hilbert for the waterfall display.\
\
Each version is at a different level of completion.\
\
Observations of each version: ( all work fairly well )\
The baseband version picks up some digital hash that is about at the level of band noise on 40 meters.  The audio image suppression is not perfect up to about 900hz.\
\
The IF version does not have noticable digital hash.  Its shortcomings would be due to having an IF image as well as an audio image to suppress.  How good it works could be improved with better filters.  As is, the IF image is well suppressed.  It has an audio image that could be suppressed with better placement of the bfo on the slope of the bandpass filter.  It works very much like any superhet type receiver in this regard.\
\
The Weaver version has some digital hash as it is also at baseband.  The audio image suppression is excellent.  When changing the A/B vfo's, there is an audio ping heard where it seems the bfo frequency is passed through until the front end is back in quadrature.  The ping also occurs when keying the QCX.  



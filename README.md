# AR3 Voice Commands


# Installation

1.1 Clone the repository
```
git clone https://github.com/FIT-Cobots/Voice-Commands.git
```

2.1 Install Pocketsphinx-Python
```
git clone --recursive https://github.com/cmusphinx/pocketsphinx-python/
cd pocketsphinx-python
python setup.py install
```

2.2 If there are errors with pulseaudio and alsa header files when installing Pocketsphinx-Python
```
sudo apt install libpulse-dev
sudo apt install libasound2-dev
```

3.1 Install the Python module word2number
```
pip install word2number
```

4.1 (Optional) To build the language model (.lm) file, you will need to build [SRILM](http://www.speech.sri.com/projects/srilm/download.html), following installation instructions from the given link.

4.2 (Optional) If there are errors with libiconv while building, use
```
make NO_ICONV=1 World
```

4.3 (Optional) Once built, cd into bin/i686-m64 (or other architecture) and run
```
./ngram-count -kndiscount -interpolate -text path/to/ar3_train-text.txt -lm path/to/ar3.lm
```

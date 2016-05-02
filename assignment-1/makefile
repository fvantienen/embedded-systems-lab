all: dsp gpp send

dsp:
	cd dsp; make remote && make fetch; cd ..

gpp:
	cd gpp; make remote && make fetch; cd ..

send:
	cd dsp; make send; cd ..
	cd gpp; make send; cd ..

.PHONY: all gpp dsp send

all: dsp gpp orig send

dsp:
	cd dsp; make remote && make fetch; cd ..

gpp:
	cd gpp; make remote && make fetch; cd ..

orig:
	cd orig; make remote && make fetch; cd ..

send:
	cd dsp; make send; cd ..
	cd gpp; make send; cd ..
	cd orig; make send; cd ..

.PHONY: all gpp dsp orig send

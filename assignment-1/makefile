REMOTE_BB=192.168.0.202
REMOTE_ESL=in4342-03@ce-eslab.ewi.tudelft.nl

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

access:
	ssh -R2230:$(REMOTE_BB):22 -L2230:localhost:2230 $(REMOTE_ESL)

.PHONY: all gpp dsp orig send access

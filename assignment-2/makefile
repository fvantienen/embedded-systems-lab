REMOTE_BB=192.168.0.202
REMOTE_ESL=in4342-03@ce-eslab.ewi.tudelft.nl
CROSS_COMPILE=1
BEAGLE_DIR=/home/root/esLAB/assignment-2

all: dsp gpp orig send

dsp:
	cd dsp; make remote && make fetch; cd ..

gpp:
	cd gpp; make remote && make fetch; cd ..

orig:
	cd orig; make remote && make fetch; cd ..

send:
	scp -r ./run.sh root@$(REMOTE_BB):$(BEAGLE_DIR)/
	cd dsp; R=$(R) make send; cd ..
	cd gpp; R=$(R) make send; cd ..
	cd orig; R=$(R) make send; cd ..

fetch:
	scp -r root@$(REMOTE_BB):$(BEAGLE_DIR)/pics/klomp.pgm_out.pgm ./
	scp -r root@$(REMOTE_BB):$(BEAGLE_DIR)/output.csv ./

access:
	ssh -R2230:$(REMOTE_BB):22 $(REMOTE_ESL)

conn:
	ssh -L2230:localhost:2230 $(REMOTE_ESL)

.PHONY: all gpp dsp orig send access conn

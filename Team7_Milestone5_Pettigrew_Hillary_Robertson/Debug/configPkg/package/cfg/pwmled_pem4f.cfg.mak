# invoke SourceDir generated makefile for pwmled.pem4f
pwmled.pem4f: .libraries,pwmled.pem4f
.libraries,pwmled.pem4f: package/cfg/pwmled_pem4f.xdl
	$(MAKE) -f C:\Users\Zachary\Documents\GitHub\robot-operator\Team7_Milestone5_Pettigrew_Hillary_Robertson/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Zachary\Documents\GitHub\robot-operator\Team7_Milestone5_Pettigrew_Hillary_Robertson/src/makefile.libs clean


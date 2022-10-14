# invoke SourceDir generated makefile for pwmled.pem4f
pwmled.pem4f: .libraries,pwmled.pem4f
.libraries,pwmled.pem4f: package/cfg/pwmled_pem4f.xdl
	$(MAKE) -f C:\Users\Aaron\DOCUME~1\GitHub\ROBOT-~1\PWMEXA~1/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Aaron\DOCUME~1\GitHub\ROBOT-~1\PWMEXA~1/src/makefile.libs clean


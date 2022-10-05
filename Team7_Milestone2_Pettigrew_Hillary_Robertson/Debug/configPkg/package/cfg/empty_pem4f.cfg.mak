# invoke SourceDir generated makefile for empty.pem4f
empty.pem4f: .libraries,empty.pem4f
.libraries,empty.pem4f: package/cfg/empty_pem4f.xdl
	$(MAKE) -f C:\Users\Zachary\Documents\GitHub\robot-operator\Team7_Milestone2_Pettigrew_Hillary_Robertson/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Zachary\Documents\GitHub\robot-operator\Team7_Milestone2_Pettigrew_Hillary_Robertson/src/makefile.libs clean


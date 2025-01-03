################################################################################
################################ - MAKEFILE RULES - ############################
################################################################################

CC			:= $(ccompiler)
CXX			:= $(compiler)
mex			:= $(matlabrootdir)/bin/mex
cudacompiler	:= $(cudarootdir)/bin/nvcc


all	: $(exetarget) $(rtexetarget) $(steptarget) $(precisiontarget)    library 

.PHONY         : $(exetarget)
$(exetarget) : $(exe-objects)
	@echo compiler path = ${compiler}
	@echo
	@echo ------------------ making executable
	@echo
	@mkdir -p $(bindir)
	$(compiler) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY         : $(rtexetarget)
$(rtexetarget) : $(rtexe-objects)
	@echo compiler path = ${compiler}
	@echo
	@echo ------------------ making real time executable
	@echo
	@mkdir -p $(bindir)
	$(compiler) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY         : $(steptarget)
$(steptarget) : $(step-objects)
	@echo compiler path = ${compiler}
	@echo
	@echo ------------------ making step-by-step example
	@echo
	@mkdir -p $(bindir)
	$(compiler) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY         : $(precisiontarget)
$(precisiontarget) : $(precision-objects)
	@echo compiler path = ${compiler}
	@echo
	@echo ------------------ making precision example
	@echo
	@mkdir -p $(bindir)
	$(compiler) $(CXXFLAGS) $^ $(LDFLAGS) -o $@
	
.PHONY         : $(robottarget)
$(robottarget) : $(robot-objects)
	@echo compiler path = ${compiler}
	@echo
	@echo ------------------ making robotsimulator example
	@echo
	@mkdir -p $(bindir)
	$(compiler) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY		: mex
mex	: $(mextarget) 
	@echo
	@echo ------------------ making mex file $(mextarget)
	@echo

$(mextarget)	: $(mex-objects)
	@echo compiler path = ${mex}
	@echo
	@echo ------------------ making mexfile
	@echo
	@mkdir -p $(mexdir)
	$(mex) $(MEXFLAGS) $^ -output $@
	
.PHONY		: sfunction
sfunction	: $(sfunctiontarget) 
	@echo
	@echo ------------------ making sfunction file $(sfunctiontarget)
	@echo

$(sfunctiontarget)	: $(sfunction-objects)
	@echo compiler path = ${mex}
	@echo
	@echo ------------------ making sfunction file
	@echo
	@mkdir -p $(sfunctiondir)
	$(mex) $(MEXFLAGS) $^ -output $@

.PHONY  : library
library : $(libtarget)
	@echo
	@echo ------------------ making library $(libtarget)
	@echo

$(libtarget): $(objects)
	@echo
	@echo ------------------ creating library
	@echo
	@mkdir -p $(libdir)
	$(AR) $(ARFLAGS) $@ $^

.PHONY : dox
dox    : Doxyfile
	@echo
	@echo ------------------ creating documentation
	@echo
	@doxygen Doxyfile

.PHONY   : doxclean
doxclean :
	@echo
	@echo ------------------ removing documentation
	@echo
	@rm -rf doc

.PHONY : distclean
distclean  :
	@echo
	@echo ------------------ cleaning everything
	@echo
	@rm -f $(pkgconfigfile) $(libtarget) $(packagename) $(objects) ${exetarget}.exe ${exe-objects} ${steptarget}.exe ${step-objects} ${precisiontarget}.exe ${precision-objects} $(dependencies) ${exe-dependencies} ${robottarget} ${robot-objects} ${robot-dependencies} ${mextarget} ${mex-objects} ${mex-dependencies} ${sfunctiontarget} ${sfunction-objects} ${sfunction-dependencies} TAGS gmon.out

.PHONY : clean
clean  :
	@echo
	@echo ------------------ cleaning *.o exe lib
	@echo
	@rm -f $(objects) ${exe-objects} ${libtarget} ${exetarget}.exe ${mextarget} ${mex-objects} ${sfunctiontarget} ${sfunction-objects} TAGS gmon.out

.PHONY : clear
clear :
	@rm -rf \#* ${dependencies}

.PHONY: install
install: $(libtarget) pkgfile uninstall install-exe
	@echo
	@echo ------------------ installing library and header files
	@echo
	@echo ------------------ installing at $(installdir)
	@echo
	@mkdir -p $(installdir)/include/$(packagename)
	@cp -vfR $(includedir)/[!.]* $(installdir)/include/$(packagename)
	@mkdir -p $(installdir)/lib/pkgconfig
	@cp -vfR $(libtarget)  $(installdir)/lib
	@echo
	@echo ------------------ installing the pkg-config file to $(installdir)/lib/pkgconfig. \
		Remember to add this path to your PKG_CONFIG_PATH variable
	@echo
	@cp $(pkgconfigfile) $(installdir)/lib/pkgconfig/

.PHONY: install-exe
install-exe: $(exetarget)
	@cp $(exetarget) $(installdir)/bin
	@cp $(rtexetarget) $(installdir)/bin

.PHONY: install-dev
install-dev : $(libtarget) pkgfile uninstall
	@echo
	@echo ------------------ installing library and development files
	@echo
	@echo ------------------ installing at $(installdir)
	@echo
	@mkdir -p $(installdir)/include/$(packagename)
	@cp -vfR $(includedir)/$(packagename)/[!.]* $(installdir)/include/$(packagename)
	@mkdir -p $(installdir)/lib/pkgconfig
	@cp -vfR $(libtarget)  $(installdir)/lib                 # copy the static library
	@mkdir -p $(installdir)/src/$(packagename)                 # create the source directory
	@cp -vfR $(srcdir)/*.c* $(installdir)/src/$(packagename) # copy development files
	@cp -vf makefile $(installdir)/src/$(packagename)
	@cp $(pkgconfigfile) $(installdir)/lib/pkgconfig/

.PHONY: uninstall
uninstall:
	@echo
	@echo ------------------ uninstalling if-installed
	@echo
	@rm -rf $(installdir)/include/$(packagename)
	@rm -f   $(installdir)/$(libtarget)
	@rm -rf $(installdir)/src/$(packagename)
	@rm -f   $(installdir)/lib/pkgconfig/$(pkgconfigfile)
	@rm -f   $(installdir)/bin/$(packagename)

ifneq "$(MAKECMDGOALS)" "clean"
  include $(dependencies)
endif


# %.d : %.cc
# 	@echo
#	@echo ------------------ compiling and creating dependencies for cc file $@
#	@echo
#	$(compiler) -c $(CFLAGS) $< | \
#	$(compiler) $(CFLAGS) -MM $< | \
#	sed 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp
#	mv -f $@.tmp $@
#	@echo

%.d : %.c
	@echo
	@echo ------------------ creating dependencies for c file $@
	@echo
	$(ccompiler) $(CXXFLAGS) -MM $< | \
	sed 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp
	mv -f $@.tmp $@
	@echo

%.d : %.cpp
	@echo
	@echo ------------------ creating dependencies for cpp file $@
	@echo
	$(compiler) $(CXXFLAGS) -MM $< | \
	sed 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp
	mv -f $@.tmp $@
	@echo
	
%.d : %.cu
	@echo
	@echo ------------------ creating dependencies for cuda file $@
	@echo
	$(cudacompiler) $(NVCCFLAGS) -M $< | \
	sed 's,\($(notdir $*)\.o\) *:-,$(dir $@)\1 $@: ,' > $@.tmp
	mv -f $@.tmp $@
	@echo
	
%.o : %.c
	@echo
	@echo ------------------ compiling c file $@
	@echo
	$(ccompiler) -c $(CXXFLAGS) $< -o $@
	@echo
	
%.o : %.cpp
	@echo
	@echo ------------------ compiling cpp file $@
	@echo
	$(compiler) -c $(CXXFLAGS) $< -o $@
	@echo
	
%.o : %.cu
	@echo
	@echo ------------------ compiling cuda file $@
	@echo
	$(cudacompiler) -c $(NVCCFLAGS) $< -o $@
	@echo


.PHONY : pkgfile
pkgfile:
	@echo
	@echo ------------------ creating pkg-config file
	@echo
	@echo "# Package Information for pkg-config"    >  $(pkgconfigfile)
	@echo "# Author: $(author)" 			>> $(pkgconfigfile)
	@echo "# Created: `date`"			>> $(pkgconfigfile)
	@echo "# Licence: $(licence)"			>> $(pkgconfigfile)
	@echo 						>> $(pkgconfigfile)
	@echo prefix=$(installdir)       		>> $(pkgconfigfile)
	@echo exec_prefix=$$\{prefix\}     		>> $(pkgconfigfile)
	@echo libdir=$$\{exec_prefix\}/lib 		>> $(pkgconfigfile)
	@echo includedir=$$\{prefix\}/include   	>> $(pkgconfigfile)
	@echo 						>> $(pkgconfigfile)
	@echo Name: "$(packagename)" 			>> $(pkgconfigfile)
	@echo Description: "$(description)" 		>> $(pkgconfigfile)
	@echo Version: "$(version)" 			>> $(pkgconfigfile)
	@echo Libs: -L$$\{libdir} -l$(packagename) 	>> $(pkgconfigfile)
	@echo Cflags: -I$$\{includedir\} 		>> $(pkgconfigfile)
	@echo 						>> $(pkgconfigfile)

.PHONY : flags
flags :
	@echo
	@echo ------------------ build flags
	@echo
	@echo ldflags  = $(LDFLAGS)
	@echo cxxflags = $(CXXFLAGS)
	@echo mexflags = $(MEXFLAGS)
	@echo nvccflags = $(NVCCFLAGS)
	@echo sources = ${sources}
	@echo objects = ${exe-objects}


.PHONY : rules
rules :
	@echo
	@echo ------------------ legitimate rules
	@echo
	@echo "(nothing)   : makes the executable : by default src/main.cpp is included to the sources list"
	@echo "              and used in the exe-build. Change its value with $exe-source-file variable"
	@echo "library     : generates the library"
	@echo "dox         : generates the doxygen documentation if Doxyfile exists"
	@echo "doxclean    : cleans up the documentation"
	@echo "clean       : cleans up .o lib and exe files"
	@echo "distclean   : cleans everything except source+headers"
	@echo "install     : installs the library"
	@echo "install-dev : installs the library along with documentation files"
	@echo "install-exe : installs the executable"
	@echo "uninstall   : uninstalls the library"
	@echo "pkgfile     : generates the pkg-config file"
	@echo "flags       : shows the flags that will be used"
	@echo "rules       : shows this text"
	@echo "clear       : clears #* & dependency files"
	@echo



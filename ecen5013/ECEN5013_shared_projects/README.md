# ECEN5013_shared_projects
**Ben Heberlein, Jeff Venicx**
###########################################################
This repository is for shared work on projects for ECEN5013 
Embedded Software Essentials

NOTE: For project 2, the normal build system in Kinetis
Design Studio was used. This makefile was only used to 
cross compile for the BBB.


# Folder Structure
###########################################################
The 'src' directory contains all c source files. This is
where you would find things like 'data.c', 'memory.c', etc.

The 'include' directory contains all header files.

The 'test' directory contains functions for all unit tests.

The 'lib' directory will contain any libraries that need to
be linked in.

Output files will be found either in the 'build' folder for
intermediate files, archives, dependency files, assembly
files, object files, etc, or in the 'bin' folder for 
finished executables.

# Makefile Operations
###########################################################

**Targets**
    The makefile can be used in the following ways.

    make build
        This is the default target and will run when you 
        input 'make'. This builds all object files, links,
        and outputs an executable. Object, map, and  
        dependency files are generated in the build folder.
    
    make compile-all
        This compiles all object files but does not link.
        Object files and dependency files are generated in
        the build folder. You can also compile any
        individual object file with 'make build/example.o' 
        or 'make example.o'. These map to the same
        command.

    make preprocess
        This performs only the preprocessing and generates
        the preprocessed output in the build folder. You
        can generate the output of a single preprocessed
        file by inputting 'make build/example.i' or by 
        inputting 'make example.i'. 

    make asm-file
        This generates assembly output of all files. You
        can also generate the output of a single assembly
        file by inputting 'make build/example.s' or by
        inputting 'make example.s'.
        
    make build-lib
        This generates a library from the library sources
        defined in 'sources.mk'.

    make upload
        This takes an executable and copies it with scp
        to a release directory on the Beagle Bone Black.
        The host address and folder of the Beagle Bone
        Black can be configured with the HOST_ADDR
        variable. 

    make dump
        This takes the final output executable and performs
        an objdump command on the output.

    make clean
        This removes all generated files in the 'build' 
        and 'bin' directories.

    make test
        This compiles all test sources and library sources
        defined in the 'sources.mk' file, generates a test
        executable in 'bin', and runs the tests. If tests
        are successful, you should see 'ALL TESTS PASSED'
        and the number of tests run.

**Input Options**
    The Makefile contains several configurable options.
    These will be input as 'OPTION=VALUE' in the make
    command.
  
    PROJECT
        This object sets compile time switches for 
        the project version we are compiling for.
    
        Possible values: 1, 2, 3, 4
        Default value: 1

    PLATFORM
        This option selects which architecture to compile
        for. This will change the compiler used and may
        change options or source files in the compilation.
        
        Possible values: HOST, BBB, FRDM
        Default value: HOST 

    HOST_ADDR
        This option specifies the host address for the 
        'make upload' command.
        
        Possible values: any valid user@host.
        Default value: root@192.168.7.2:

    DEBUG
        This option sets the flags -g and -O0 for gcc.
        
        Possible values: TRUE, FALSE
        Default value: FALSE

    WARNINGS
        This option sets the flags -Wall and -Wextra for
        the gcc compiler.

        Possible values: TRUE, FALSE
        Default value: FALSE

    FLAGS
        This option allows the user to pass in multiple 
        extra gcc flags. These will be added to the 
        build command.

        Possible values: any valid gcc flag
        Default value: NONE



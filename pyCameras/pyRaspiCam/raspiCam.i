
%module raspiCam


%{
#define SWIG_FILE_WITH_INIT


 
 /* Includes the header in the wrapper code */
 #include "raspiCam.h"
 
 %}
 

%include "typemaps.i" #TODO
%include "numpy.i"

%init %{
import_array();

%}

%apply (unsigned char* INPLACE_ARRAY3, int DIM1, int DIM2, int DIM3) {(unsigned char* seq, int n1, int n2, int n3)};
%apply (unsigned char* INPLACE_ARRAY2, int DIM1, int DIM2) {(unsigned char* seq, int n1, int n2)};
%apply (unsigned short* INPLACE_ARRAY2, int DIM1, int DIM2) {(unsigned short* seq, int n1, int n2)};
%apply (unsigned short* INPLACE_ARRAY3, int DIM1, int DIM2, int DIM3) {(unsigned short* seq, int n1, int n2, int n3)};


 
 
%include "raspiCam.h"



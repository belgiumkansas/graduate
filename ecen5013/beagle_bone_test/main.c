
#include<stdio.h>
#include "hw1.c"



void main(int argc, const char ** argv){
  ///must ensure reverse string is non-literal!!!
  //cannot detect non literals from within reverse
  //get seg faults for literal strings
  char str1[] = "This is a string";
  int length = 16;
  char error;

  printf("%s<->", str1);
  error = reverse(str1, length);
  printf("%s\nreturn code:%c\n\n", str1, error);

  char str2[] = "some NUMmbers12345";
  length = 18;
  error;

  printf("%s<->", str2);
  error = reverse(str2, length);
  printf("%s\nreturn code:%c\n\n", str2, error);

  char str3[]="Does it reverse \\n\\0\\t correctly?";
  length = 33;
  error;

  printf("%s<->", str3);
  error = reverse(str3, length);
  printf("%s\nreturn code:%c\n", str3, error);
}

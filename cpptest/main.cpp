#include <iostream>
//#include <string>
#include "sample.h"

using namespace std;

int main() {
//  int a;
//  cin >> a;
//  cout << "a=" << a << endl;

//  string s,t;
//  t = "入力された文字は、";
//  cout << "文字列を入力:";
//  cin >> s;
//  cout << t+s << "です。" << endl;

  CSample obj;
  int num;

  cout << "整数を入力してください:" << endl;
  cin >> num;

  obj.set( num );
  cout << obj.get() << endl;

  return 0;
}
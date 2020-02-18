[TOC]

# Tips

### [模板类实例化](https://zhuanlan.zhihu.com/p/77241939)

C++ 的函数模板本质上函数的重载，泛型只是简化了程序员的工作，让这些重载通过编译器来完成。

隐式实例化可能影响效率,所以需要提高效率的显式实例化，显式实例化在编译期间就会生成实例（增加了编译时间）。在**函数模板定义后**，我们可以通过显式实例化的方式告诉编译器生成指定实参的函数。显式实例化声明会阻止隐式实例化。如果我们在显式实例化时，只指定部分模板实参，则指定顺序必须自左至右依次指定，不能越过前参模板形参，直接指定后面的。

函数模板可以做为类的成员函数。

需要注意的是：*函数模板不能用作虚函数*。这是因为 C++编译器在解析类的时候就要确定虚函数表(vtable)的大小，如果允许一个虚函数是函数模板，那么就需要在解析这个类之前扫描所有的代码，找出这个模板成员函数的调用或显式实例化操作，然后才能确定虚函数表的大小，而显然这是不可行的。

函数模板之间、普通函数和模板函数之间可以重载。编译器会根据调用时提供的函数参数，调用能够处理这一类型的最佳匹配版本。在匹配度上，一般按照如下顺序考虑：

+ 最符合函数名和参数类型的普通函数
+ 特殊模板(具有非类型形参的模板，即对T有类型限制)
+ 普通模板(对T没有任何限制)
+ 通过类型转换进行参数模板匹配的重载函数

使用`template class`进行显示实例化

```cpp
template class ClassifyGrPro<pcl::PointXYZI>;
template class ClassifyGrPro<pcl::PointXYZINormal>;
```

**函数模板特化 specialization**

函数模板特化的本质是实例化一个模板，而非重载它。因此，特化不影响编译器函数匹配。

函数模板及其特化版本应该声明在同一个头文件中。所有同名模板的声明应该放在前面，然后是这些模板的特化版本。

程序运行结果和使用函数模板特化相同。但是，使用普通函数重载和使用模板特化还是有不同之处，主要表现在如下两个方面：

（1）如果使用普通重载函数，那么不管是否发生实际的函数调用，都会在目标文件中生成该函数的二进制代码。而如果使用模板的特化版本，除非发生函数调用，否则不会在目标文件中包含特化模板函数的二进制代码。这符合函数模板的“惰性实例化”准则。

（2）如果使用普通重载函数，那么在分离编译模式下，应该在各个源文件中包含重载函数的申明，否则在某些源文件中就会使用模板函数，而不是重载函数。

#### [形参包](https://zh.cppreference.com/w/cpp/language/parameter_pack)

在主类模板中，模板形参包必须是模板形参列表的最后一个形参。在函数模板中，模板参数包可以在列表中稍早出现，只要其后的所有形参均可从函数实参推导或拥有默认实参即可：

```cpp
template<typename... Ts, typename U> struct Invalid; // 错误：Ts.. 不在结尾
 
template<typename ...Ts, typename U, typename=void>
void valid(U, Ts...);     // OK：能推导 U
// void valid(Ts..., U);  // 不能使用：Ts... 在此位置是非推导语境
 
valid(1.0, 1, 2, 3);      // OK：推导 U 为 double，Ts 为 {int,int,int}
```

SFINAE是英文Substitution failure is not an error的缩写，意思是匹配失败不是错误。这句话什么意思呢？当调用模板函数时编译器会根据传入参数推导最合适的模板函数，在这个推导过程中如果某一个或者某几个模板函数推导出来是编译无法通过的，只要有一个可以正确推导出来，那么那几个推导得到的可能产生编译错误的模板函数并不会引发编译错误。

```cpp
struct Test {
    typedef int foo;
};

template <typename T> 
void f(typename T::foo) {} // Definition #1

template <typename T> 
void f(T) {}               // Definition #2

int main() {
    f<Test>(10); // Call #1.
    f<int>(10);  // Call #2. Without error (even though there is no int::foo) thanks to SFINAE.
}
```



### [C++11中const和constexpr](https://zhuanlan.zhihu.com/p/20206577)

C＋＋11中新增加了用于指示常量表达式的constexpr关键字,
![aca0fcf4](img/aca0fcf4.png)

c++11中constexpr包含了const含义,non-static data members, static constexpr data members, and static const data members of integral or enumeration type may be initialized in the class declaration.

  ```cpp
  struct X 
  {
      int i=5;
      const float f=3.12f;
      static const int j=42;
      static constexpr float g=9.5f;
  };
  ```
![d0e94a01](img/d0e94a01.png)

### [lambda表达式](https://www.cnblogs.com/pzhfei/archive/2013/01/14/lambda_expression.html)
[Lambda 表达式 (C++11 起) - cppreference.com](https://zh.cppreference.com/w/cpp/language/lambda)

```
[capture](parameters)->return-type{body}
```
如果没有参数,空的圆括号()可以省略.返回值也可以省略,如果函数体只由一条return语句组成或返回类型为void的话.形如:
```
[capture](parameters){body}
```
```cpp
[](int x, int y) { return x + y; } // 隐式返回类型
[](int& x) { ++x; }   // 没有return语句 -> lambda 函数的返回类型是'void'
[]() { ++global_x; }  // 没有参数,仅访问某个全局变量
[]{ ++global_x; }     // 与上一个相同,省略了()
```
Lambda函数可以引用在它之外声明的变量. 这些变量的集合叫做一个闭包. 闭包被定义在Lambda表达式声明中的方括号[]内. 这个机制允许这些变量被按值或按引用捕获:
```cpp
[]        //未定义变量.试图在Lambda内使用任何外部变量都是错误的.
[x, &y]   //x 按值捕获, y 按引用捕获.
[&]       //用到的任何外部变量都隐式按引用捕获
[=]       //用到的任何外部变量都隐式按值捕获
[&, x]    //x显式地按值捕获. 其它变量按引用捕获
[=, &z]   //z按引用捕获. 其它变量按值捕获
```
对this的捕获比较特殊, 它只能按值捕获. this只有当包含它的最靠近它的函数不是静态成员函数时才能被捕获.对protect和priviate成员来说, 这个lambda函数与创建它的成员函数有相同的访问控制. 如果this被捕获了,不管是显式还隐式的,那么它的类的作用域对Lambda函数就是可见的. 访问this的成员不必使用this->语法,可以直接访问.
一个没有指定任何捕获的lambda函数,可以显式转换成一个具有相同声明形式函数指针.所以,像下面这样做是合法的:

```cpp
auto a_lambda_func = [](int x) { /*...*/ };
void(*func_ptr)(int) = a_lambda_func;
func_ptr(4); //calls the lambda.
```

### [extern的使用](https://www.cnblogs.com/broglie/p/5524932.html)

**extern一般是使用在多文件之间需要共享某些代码**.

- 变量的生明和定义

  一个文件的代码可能需要另一个文件中中定义的变量,定义则负责创建与名字关联的实体，定义还申请存储空间。

  ```cpp
  extern int i;  //声明i
  int j;         //声明并定义i
  ```

  变量能且只能被定义一次，但是可以被声明多次。

+ 多文件共享const对象

  对于const变量不管是声明还是定义都添加extern关键字：

  ```cpp
  //file1.cpp定义并初始化和一个常量，该常量能被其他文件访问
  extern const int bufferSize = function();
  //file.h头文件
  extern const int bufferSize; //与file1.cpp中定义的是同一个
  ```

+ 模板的控制实例化

  当两个或者多个独立编译的源文件中使用了相同的模板并且提供了相同的模板参数时，每个文件中都会有该模板的一个实例。在多个文件中实例化相同的模板的额外开销可能非常严重，在C++11新标准中，我们可以通过显式实例化来避免这种开销。一个显式实例化具有如下形式：

  ```cpp
  extern template declaration; //实例化声明
  template declaration;        //实例化定义
  extern template class vec<string>;       //声明
  template int sum(const int, const int);  //定义
  ```

  当编译器遇到extern模板声明时，它不会在本文件中生成实例化代码，将一个实例化声明为extern就表示承诺在程序的其他位置有该实例化的一个非extern定义。

### [extern “C”的作用详解](https://www.cnblogs.com/carsonzhu/p/5272271.html)

   extern "C"的主要作用就是为了能够正确实现C++代码调用其他C语言代码。加上extern "C"后，会指示编译器这部分代码按C语言（而不是C++）的方式进行编译。由于C++支持函数重载，因此编译器编译函数的过程中会将函数的参数类型也加到编译后的代码中，而不仅仅是函数名；而C语言并不支持函数重载，因此编译C语言代码的函数时不会带上函数的参数类型，一般只包括函数名。

```cpp
   #ifdef __cplusplus             //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
   extern "C"{
   #endif
    
   /*…*/
    
   #ifdef __cplusplus
   }
   #endif
```

   被extern "C"限定的函数或变量是extern类型的, extern是C/C++语言中表明函数和全局变量的作用范围的关键字，该关键字告诉编译器，其申明的函数和变量可以在本模块或其他模块中使用。**extern对应的关键字是static，static表明变量或者函数只能在本模块中使用，因此，被static修饰的变量或者函数不可能被extern C修饰。**

### [文件读写](https://www.cnblogs.com/liaocheng/p/4371796.html)

头文件：#include <fstream>

fstream提供三种类，实现C++对文件的操作

ofstream：写操作，由ostream引申而来

ifstream：读操作，由istream引申而来 

fstream ：同时读写操作，由iostream引申而来 

文件的类型：文本文件 和 二进制文件

写操作：<<,put( ), write( ) 读操作： >> , get( ),getline( ), read( )

#### 文本文件的读写

##### 一次性读写若干个字符

+ 使用运算符<< 和 >>进行读写

  `<< `能实现以行为单位写入文件

  `>>` 不能一行为单位读入内存,总是以空格、Tab、回车结束，而是以单词为单位

+ 使用运算符<<(写)和getline()进行读写

   <<：以行为单位输入文件

   getline()：以行为单位 读入内存，能一次读入一行

   函数原型：istream &getline( char *buffer, streamsize num );

  功能：getline( )函数用于从文件读取num-1个字符到buffer(内存)中，直到下列情况发生时，读取结束:

  - num - 1个字符已经读入
  - 碰到一个换行标志
  - 碰到一个EOF

  `OpenFile.getline(str,20);` // 满足上面3个条件中的一个就返回

  也可使用`std::getline((std::istream&, std::string& str, char delim))`, `std::getline((std::istream&, std::string& str))`

  ```cpp
  bool readPose(const std::string& pose_file, std::vector<unsigned int>& frame_id, std::vector<std::vector<float> >& pose)
  {
    std::ifstream in_file;
    in_file.open(pose_file, std::ifstream::in);
    if (!in_file.is_open())
    {
      std::cout << "\033[0;33m can not open pose file: " << pose_file << "\033[0m" << std::endl;
      return false;
    }
    std::string line;
    while (std::getline(in_file, line))
    {
      if (line.length() == 0)
      {
        continue;  // ignore blank lines
      }
      std::istringstream ss(line);
      std::string str_tmp;
      ss >> str_tmp;
      frame_id.emplace_back(static_cast<unsigned int>(std::stoi(str_tmp)));
      std::vector<float> pose_tmp;
      for (std::size_t j = 0; j < 6; ++j)
      {
        ss >> str_tmp;
        pose_tmp.emplace_back(std::stof(str_tmp));
      }
      pose.emplace_back(pose_tmp);
    }
    in_file.close();
    return true;
  }
  ```

##### 一次读写一个字符

使用get( )和put( )函数

函数声明：istream& get(char &c);

函数功能：使用 get( )函数 把字符1输入到文件

#### 二进制文件的读写

##### 使用read()和write()进行读写

read( ):

功能：从文件中提取 n 个字节数据，写入buf指向的地方中

函数声明：istream &  read ( char * buf ,  int  n ) ;

write( ):

功能：把buf指向的内容取n个字节写入文件

函数声明：ostream & ostream :: write ( char * buf ,  int  n ) ;

该函数遇到空字符时并不停止，因而能够写入完整的类结构,第一个参数一个char型指针（指向内存数据的起始地址），与对象结合使用的时候，要在对象地址之前要char做强制类型转换。

gcount()函数经常和read函数配合使用，用来获得实际读取的字节数。

```cpp
//写文件：二进制存储1234  
　　int writeNum1 = 1;  
　　int writeNum2 = 2;  
　　int writeNum3 = 3;  
　　int writeNum4 = 4;  
    ofstream fout("test.txt", ios::out | ios::binary);  
    fout.write(reinterpret_cast<char *>(&writeNum1), sizeof(int));  
    fout.write(reinterpret_cast<char *>(&writeNum2), sizeof(int));  
    fout.write(reinterpret_cast<char *>(&writeNum3), sizeof(int));  
    fout.write(reinterpret_cast<char *>(&writeNum4), sizeof(int));  
    fout.close();  
ifstream fin("test.txt",ios::in | ios::binary);  

　　if (!fin.good())  

    {  
        cout<<"文件打开错误"<<endl;    
        exit(0);  
    }  
　　int readNum = 0; 
//第一次输出:从第一个数字输出，结果是1 2 3 4  
    fin.seekg(0,ios::beg);  
　　while (fin.peek() != EOF)  
    {  
        fin.read(reinterpret_cast<char*>(&readNum), sizeof(int));  
        cout<<readNum<<" ";  
    }  
```



##### 使用运算符get( ) 和 put( )读写一个字节

get( ) ：在文件中读取一个字节到内存

函数原型：ifstream &get(char ch)

put( ) ：在内存中写入一个字节到文件

函数原型：ofstream &put(char ch)

程序不再使用文件时，为什么要关闭文件:

+ 文件缓冲区是一块小的内存空间
+ 操作系统限制同时打开的文件数量

文件的默认打开方式为文本文件，要是想以二进制的方式处理，在打开时要用 ios::binary 显式声明。针对文本文件操作时，get函数和>>的区别：

+ 在读取数据时，get函数包括空白字符（遇空白字符不停止读取）

+ `>>`在默认情况下拒绝接受空白字符（遇到空白符停止读取）

![1561082565808](img/1561082565808.png)



### [typedef vs define](https://softwareengineering.stackexchange.com/questions/130679/typedefs-and-defines)

A `typedef` is generally preferred unless there's some odd reason that you specifically need a macro. 推荐使用`typedef`，macro做文本替换，typedef给其一个别名。

```cpp
typedef char *char_ptr;
char_ptr a, b;
#define CHAR_PTR char*
CHAR_PTR c, d;
// a, b, and c are all pointers, but d is a char, because the last line expands to:
char *c;
char d;
```



### [#pragma once vs #ifndef](https://blog.csdn.net/WInScar/article/details/7016146)

https://stackoverflow.com/questions/1143936/pragma-once-vs-include-guards

在能够支持这两种方式的编译器上，二者并没有太大的区别，但是两者仍然还是有一些细微的区别。    

方式一：

​    \#ifndef __SOMEFILE_H__
​    \#define __SOMEFILE_H__
​    ... ... // 一些声明语句
​    \#endif

 #ifndef的方式依赖于宏名字不能冲突，这不光可以保证同一个文件不会被包含多次，也能保证内容完全相同的两个文件不会被不小心同时包含。当然，缺点就是如果不同头文件的宏名不小心“撞车”，可能就会导致头文件明明存在，编译器却硬说找不到声明的状况.

方式二：

​    \#pragma once
​    ... ... // 一些声明语句

#pragma once则由编译器提供保证：同一个文件不会被包含多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。带来的好处是，你不必再费劲想个宏名了，当然也就不会出现宏名碰撞引发的奇怪问题。对应的缺点就是如果某个头文件有多份拷贝，本方法不能保证他们不被重复包含。当然，相比宏名碰撞引发的“找不到声明”的问题，重复包含更容易被发现并修正。

方式一 由语言支持所以移植性好，方式二 可以避免名字冲突。




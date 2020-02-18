[TOC]

# 高质量程序设计指南　－－　C++/C语言 林悦



# C++11新特性解析与应用



### 右值

如果临时对象通过一个接受右值的函数传递给另一个函数时，就会变成左值，因为这个临时对象在传递过程中，变成了命名对象。

```cpp
void process_value(int& i) { 
 std::cout << "LValue processed: " << i << std::endl; 
} 
 
void process_value(int&& i) { 
 std::cout << "RValue processed: " << i << std::endl; 
} 
 
void forward_value(int&& i) { 
 process_value(i); 
} 
 
int main() { 
 int a = 0; 
 process_value(a); 
 process_value(1); 
 forward_value(2); 
}
```

运行结果:

```cpp
LValue processed: 0 
RValue processed: 1 
LValue processed: 2
```

虽然 2 这个立即数在函数 forward_value 接收时是右值，但到了 process_value 接收时，变成了左值。C++11 中定义的 T&& 的推导规则为：右值实参为右值引用，左值实参仍然为左值引用,一句话，就是参数的属性不变。这样也就完美的实现了参数的完整传递。

在设计类的时候如果有动态申请的资源，也应该设计转移构造函数和转移拷贝函数。在设计类库时，还应该考虑 std::move 的使用场景并积极使用它。
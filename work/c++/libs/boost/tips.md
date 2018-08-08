
### 控制子线程顺序执行
```cpp
boost::thread thread(threadFun);
```
当一个thread执行完成时，这个子线程就会消失。注意这个线程对象不会消失，它仍然是一个还处在它的生存期的C++对象。同理，当对一个堆上的线程对象的指针调用delete时候，线程对象被销毁，操作系统的线程并不能保证就消失。  
`thread.join()`让调用这个方法threadFun的线程thread进入wait状态，直到函数调用完成为止，join是一个等待子线程结束的最好的方法。[join()的方法之后的线程变量thread也就不能被调用，因为这个变量不再是一个有效的线程](https://blog.csdn.net/huang_xw/article/details/8453660)。任何一个函数内可以做的事情也可以在一个线程内完成。 归根结底，一个线程只不过是一个函数，除了它是同时执行的。

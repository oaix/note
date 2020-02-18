
# Tips

## [Thread](https://www.boost.org/doc/libs/1_61_0/doc/html/thread/thread_management.html#thread.thread_management.thread)
由于每一个boost::thread对应一个线程，所以创建线程就是创建一个boost::thread对象。thread构造函数声明接受的是一个没有参数且返回类型为void的型别.
```cpp
boost::thread thread(CallableFun);
```
这里需要一个函数对象(函数指针，仿函数):
+ 仿函数
  ```cpp
  struct callable
  {
      void operator()(void)
      {
           std::cout << "In callable ..." << std::endl;
      }
  };
  boost::thread thread_call(struct callable);
  ```
+ 全局函数

+ 类成员函数
  利用`boost::bind`
  ```cpp
  thread_test athread_test;
  boost::thread thread_member_call(boost::bind(&thread_test::invoke,&athread_test));
  ```
+ 含参函数对象
  ```cpp
  void func_arg(int num)
  {
       std::cout << "In func_arg ..." << " num=" << num << std::endl;
  }
  boost::thread thread_arg_bind(boost::bind(&func_arg,1012));
  boost::thread thread_arg(func_arg,2012);
  ```
### 控制子线程顺序执行
当一个thread执行完成时，这个子线程就会消失。注意这个线程对象不会消失，它仍然是一个还处在它的生存期的C++对象。同理，当对一个堆上的线程对象的指针调用delete时候，线程对象被销毁，操作系统的线程并不能保证就消失。  
`thread.join()`让调用这个方法threadFun的线程thread进入wait状态，直到函数调用完成为止，join是一个等待子线程结束的最好的方法。[join()的方法之后的线程变量thread也就不能被调用，因为这个变量不再是一个有效的线程](https://blog.csdn.net/huang_xw/article/details/8453660)。任何一个函数内可以做的事情也可以在一个线程内完成。 归根结底，一个线程只不过是一个函数，除了它是同时执行的。
+ sleep_for
  ```cpp
  // boost::this_thread::sleep(boost::posix_time::seconds(2));已弃用
  boost::this_thread::sleep_for(boost::chrono::seconds(60));
  //主线程
  boost::thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));
  ```

+ [yield vs sleep](https://stackoverflow.com/questions/2668546/multithreading-when-to-yield-versus-sleep)
sleep and yield is not the same. When calling sleep the process/thread gives CPU to another process/thread for the given amount of time.
yield relinquishes the CPU to another thread, but may return immediately if there are no other threads that waits for CPU.
So if you want to throttle, for example when streaming data at regular intervals, then sleep or nanosleep is the function to use.

### mutex
boost::mutex是最基础的锁，有lock和unlock方法，可以认为是互持锁。boost::shared_mutex是共享锁，有lock、unlock方法以及shared_lock、shared_unlock方法。boost::recursive_mutex是重入锁或者称为递归锁.
[read_write_lock](https://stackoverflow.com/questions/989795/example-for-boost-shared-mutex-multiple-reads-one-write)
```cpp
boost::shared_mutex _access;
void reader()
{
  boost::shared_lock< boost::shared_mutex > lock(_access);
  // do work here, without anyone having exclusive access
}

void conditional_writer()
{
  boost::upgrade_lock< boost::shared_mutex > lock(_access);
  // do work here, without anyone having exclusive access

  if (something) {
    boost::upgrade_to_unique_lock< boost::shared_mutex > uniqueLock(lock);
    // do work here, but now you have exclusive access
  }

  // do more work here, without anyone having exclusive access
}

void unconditional_writer()
{
  boost::unique_lock< boost::shared_mutex > lock(_access);
  // do work here, with exclusive access
}
```



## [filesystem](https://www.boost.org/doc/libs/1_61_0/libs/filesystem/doc/reference.html#remove_all)
+ [判断路径是否有效，文件夹是否存在](https://www.cnblogs.com/earvin/p/7456457.html)
```cpp
if (boost::filesystem::is_directory(data_path))
Returns: s.type() == directory_file
```
+ 判断文件是否存在
```cpp
boost::filesystem::exists(file);
Returns: status_known(s) && s.type() != file_not_found
// 包含is_directory的多用
bool isfileExist(const std::string& file_name)
{
  if (boost::filesystem::exists(file_name) && boost::filesystem::is_regular_file(file_name))
  {
    return true;
  }
  return false;
}
```

+ 创建文件夹
```cpp
if (!boost::filesystem::create_directory(save_path))
    
if (!boost::filesystem::is_directory(g_path))
  {
    std::cout << "\033[0;33m path doesn't exist, create it automaticallly! \033[0m" << std::endl;
    if (!boost::filesystem::create_directories(g_path))
    {
      std::cout << "\033[0;31m can not create path directory, return! \033[0m" << std::endl;
      return 0;
    }
    else
    {
      std::cout << " path: " << g_path << std::endl;
    }
  }
```

+ 文件复制
```cpp
boost::filesystem::rename(img_file, rename_img_file);
```

+ [判断文件夹中用多少文件](https://stackoverflow.com/questions/11140483/how-to-get-list-of-files-with-a-specific-extension-in-a-given-folder)
```cpp
std::size_t getFrameNumber(const std::string& pcd_path, const std::string& ext)
{
  std::size_t num = 0;
  boost::filesystem::directory_iterator itend;
  for (boost::filesystem::directory_iterator it(pcd_path); it != itend; ++it)
  {
    if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      ++num;
    }
  }
  return num;
}
```

+ [condition_variable](https://stackoverflow.com/questions/16907072/how-do-i-use-a-boost-condition-variable-to-wait-for-a-thread-to-complete-process)
```cpp
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
boost::mutex io_mutex;
bool worker_is_done = false;
boost::condition_variable condition;
void workFunction()
{
    std::cout << "Waiting a little..." << std::endl;
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    worker_is_done = true;
    std::cout << "Notifying condition..." << std::endl;
    condition.notify_one();
    std::cout << "Waiting a little more..." << std::endl;
    boost::this_thread::sleep(boost::posix_time::seconds(1));
}
int main()
{
    boost::mutex::scoped_lock lock(io_mutex);
    boost::thread workThread(&workFunction);

    while (!worker_is_done) condition.wait(lock);
    std::cout << "Condition notified." << std::endl;
    workThread.join();
    std::cout << "Thread finished." << std::endl;

    return 0;
}
```

+ 计算代码耗时
```cpp
boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::universal_time();
std::cout << "differ image cost time: " << (t2 - t1).total_milliseconds() << " ms" << std::endl;
```

+ 解析string中的字符

  ```cpp
  void findAllOccurances(std::vector<std::size_t>& vec, std::string data, std::string to_search,
                                     FindSubstr finder)
  {
    std::size_t pos = finder(data, to_search, 0);
    while (pos != std::string::npos)
    {
      vec.emplace_back(pos);
      pos = finder(data, to_search, pos + to_search.size());
    }
  }
  std::vector<std::size_t> slash_pos;
  findAllOccurances(slash_pos, topic_[i], "/", [&](std::string data, std::string to_search, std::size_t pos) {
          return data.find(to_search, pos);
        });
  ```

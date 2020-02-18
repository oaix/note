# Tips

## [multiprocessing](https://medium.com/@urban_institute/using-multiprocessing-to-make-python-code-faster-23ea5ef996ba)

其中2个类Process和Pool都可以，Process是将每个任务分配给不同的处理器(使用于任务数少，每个任务处理的时间较长,将所有任务都分配内存)，Pool将一序列任务发送给一个处理器(使用于大量任务，而每个任务耗时较短，只将正在处理的任务存入内存中)。Pool将一系列任务分配给每个核，每个核中任务都是顺序执行的,如果有较长时间的IO操作，那么核一直等待当前任务的IO操作完成，这样就造成了资源浪费。Process会暂停IO操作的任务，将运算资源分配给其他任务。
pool class works better when there are more processes and small IO wait. Process class works better when processes are small in number and IO operations are long. 

[Python Multiprocessing: Pool vs Process - Comparative Analysis - Ellicium Solutions](https://www.ellicium.com/python-multiprocessing-pool-process/)
```python

import time
import multiprocessing 

def basic_func(x):
    if x == 0:
        return 'zero'
    elif x%2 == 0:
        return 'even'
    else:
        return 'odd'

def multiprocessing_func(x):
    y = x*x
    time.sleep(2)
    print('{} squared results in a/an {} number'.format(x, basic_func(y)))
    
if __name__ == '__main__':
    starttime = time.time()
    processes = []
    for i in range(0,10):
        p = multiprocessing.Process(target=multiprocessing_func, args=(i,))
        processes.append(p)
        p.start()
    # 阻塞是为了统计所有任务完成所需要的时间
    for process in processes:
        process.join()
        
    print('That took {} seconds'.format(time.time() - starttime))
```
```python
if __name__ == '__main__':
    
    starttime = time.time()
    pool = multiprocessing.Pool(multiprocessing.cpu_count())
    pool.map(multiprocessing_func, range(0,10))
    pool.close()
    print('That took {} seconds'.format(time.time() - starttime))
```



## python基础

### [numpy中csv文件的存储与读取](https://www.cnblogs.com/sunshinewang/p/8892330.html)

#### 一维数组和二维数组

+ 写入

  `np.savetxt(frame,array,fmt='%.18e',delimiter=None,newline='\n', header='', footer='', comments='# ', encoding=None)`

  - frame : 文件、字符串或产生器，可以是.gz或.bz2的压缩文件
  - array : 存入文件的数组 （一维或者二维）
  - fmt：写入文件的格式，例如： %d %.2f %.18e
  - delimiter : 分割字符串，默认是任何空格

+ 读取

  ```python
  np.loadtxt(fname, dtype=<type 'float'>, comments='#',  delimiter=None, converters=None, skiprows=0, usecols=None, unpack=False,  ndmin=0, encoding='bytes')
  ```

  - frame : 文件、字符串或产生器，可以是.gz或.bz2的压缩文件
  - dtype : 数据类型，可选
  - delimiter : 分割字符串，默认是任何空格 
  - usecols：选取数据的列
  - unpack : 如果True，读入属性将分别写入不同变量

  ```python
  b = np.loadtxt('a.csv',dtype = np.int,delimiter=',',usecols=(0,1,2))
  
  b
  array([[ 0,  1,  2],
         [20, 21, 22],
         [40, 41, 42],
         [60, 61, 62],
         [80, 81, 82]])
  
  b = np.loadtxt('a.csv',dtype = np.int,delimiter=',',usecols=(2,))
  b
  array([ 2, 22, 42, 62, 82])
  
  b = np.loadtxt('a.csv',dtype = np.int,delimiter=',',usecols=(2))
  b
  array([ 2, 22, 42, 62, 82])
  ```

  **CSV文件只能有效存储一维和二维数组np.savetxt() np.loadtxt()只能有效存取一维和二维数组**

#### 多维数组 (任意维度)

  + 写入

    ```python
    a = np.arange(100).reshape((5,10,2))
    a.tofile('b.dat',sep=',',format='%d')
    ```

  + 读取

    ```python
    python1 1np.fromfile(frame, dtype=float, count	= ‐1, sep='')
    ```

    dtype : 读取的数据类型 。**可以发现，我们读取数据的时候都需要指定数据类型，无论是不是一维二维。默认为浮点型**

    count : 读入元素个数， ‐1表示读入整个文件

    sep : 数据分割字符串，如果是空串，写入文件为二进制

    **需要注意**

    + **该方法需要读取时知道存入文件时数组的维度和元素类型**
    + **a.tofile()和np.fromfile()需要配合使用**
    + **可以通过元数据文件来存储额外信息**

#### 文件的便捷存取

```python
np.save(fname, array) 或 np.savez(fname, array)
```

+ fname : 文件名，以.npy为扩展名，压缩扩展名为.npz
+ array : 数组变量

```python
np.load(fname)
```

+ fname : 文件名，以.npy为扩展名，压缩扩展名为.npz

**这里的存储，实际上也是二进制文件，如果只在python中进行操作，这种方法很方便，如果需要与其他程序进行交互，则需要视情况存储为CSV文件等。**



### [pandas.read_csv](https://www.cnblogs.com/datablog/p/6127000.html)

```python
pandas.read_csv(filepath_or_buffer, sep=', ', delimiter=None, header='infer', names=None, index_col=None, usecols=None, squeeze=False, prefix=None, mangle_dupe_cols=True, dtype=None, engine=None, converters=None, true_values=None, false_values=None, skipinitialspace=False, skiprows=None, skipfooter=0, nrows=None, na_values=None, keep_default_na=True, na_filter=True, verbose=False, skip_blank_lines=True, parse_dates=False, infer_datetime_format=False, keep_date_col=False, date_parser=None, dayfirst=False, iterator=False, chunksize=None, compression='infer', thousands=None, decimal=b'.', lineterminator=None, quotechar='"', quoting=0, doublequote=True, escapechar=None, comment=None, encoding=None, dialect=None, tupleize_cols=None, error_bad_lines=True, warn_bad_lines=True, delim_whitespace=False, low_memory=True, memory_map=False, float_precision=None)
```



### class

#### [类变量和实例变量](https://www.cnblogs.com/crazyrunning/p/6945183.html)

Generally speaking, instance variables are for data unique to each instance and class variables are for attributes and methods shared by all instances of the class:

实例变量是对于每个实例都独有的数据，而类变量是该类所有实例共享的属性和方法。

```python
class Dog:
    kind = 'canine'         # class variable shared by all instances
    def __init__(self, name):
        self.name = name    # instance variable unique to each instance
```

类`Dog`中，类属性`kind`为所有实例所共享；实例属性`name`为每个`Dog`的实例独有。

#### [类对象和实例对象](https://www.cnblogs.com/loleina/p/5409084.html)

+ 类对象

  `Python`中一切皆对象；类定义完成后，会在当前作用域中定义一个以类名为名字，指向类对象的名字。

  ```python
  class Dog:
      pass
  ```

  会在当前作用域定义名字`Dog`，指向类对象`Dog`.

  类对象支持的操作：

  - 实例化

    使用`instance_name = class_name()`的方式实例化，实例化操作创建该类的实例。`dog = Dog()`

  - 属性应用

    使用`class_name.attr_name`的方式引用类属性

  类对象就是可以用类名字直接使用表示的对象。对于类方法的使用，需要实例化一个对象后，将对象名赋值给self使用。

  ```python
  class test:
      data = 1
      def __init__(self):
          self.property=0
  
      def test2(self):
          print 'hello'
  if __name__=='__main__':
      t = test()
      print test.data
      print t.data
      print test.test2
      print t.test2()
      print test.test2(t)
  ```

  ```
  1
  1
  <unbound method test.test2>
  hello
  hello
  ```

+ 实例对象

  实例对象是类对象实例化的产物，实例对象仅支持一个操作: 属性应用

  与类对象属性引用的方式相同，使用`instance_name.attr_name`的方式。

按照严格的面向对象思想，所有属性都应该是实例的，类属性不应该存在。那么在`Python`中，由于类属性绑定就不应该存在，类定义中就只剩下函数定义了。

In practice, the statements inside a class definition will usually be function definitions, but other statements are allowed, and sometimes useful.

实践中，类定义中的语句通常是函数定义，但是其他语句也是允许的，有时也是有用的。

这里说的其他语句，就是指类属性的绑定语句。

#### 属性绑定

在定义类时，通常我们说的定义属性，其实是分为两个方面的：

+ 类属性绑定
+ 实例属性绑定

用**绑定**这个词更加确切；不管是类对象还是实例对象，属性都是依托对象而存在的。

#### [super](http://funhacks.net/explore-python/Class/super.html)

[super实现的运行机制如下面代码所示](https://stackoverflow.com/questions/576169/understanding-python-super-with-init-methods)

```python
class ChildB(Base):
    def __init__(self):
        mro = type(self).mro()
        for next_class in mro[mro.index(ChildB) + 1:]: # slice to end
            if hasattr(next_class, '__init__'):
                next_class.__init__(self)
                break
```

在类的继承中，如果重定义某个方法，该方法会覆盖父类的同名方法，但有时，我们希望能同时实现父类的功能，这时，我们就需要调用父类的方法了，可通过使用 `super` 来实现

+ `super` 和父类没有实质性的关联,

+ `super(cls, inst)` 获得的是 cls 在 inst 的 MRO 列表中的下一个类。

It's rather hand-wavey and doesn't tell us much, but the point of `super` is not to avoid writing the parent class. The point is to ensure that the next method in line in the method resolution order (MRO) is called. This becomes important in multiple inheritance.


# [sanitizers](https://github.com/google/sanitizers)

Sanitizers包含多个工具，其中常见的几个工具是：

- **[AddressSanitizer](https://github.com/google/sanitizers/wiki/AddressSanitizer#introduction)**：检测内存错误，例如释放已经释放的内存、再次access已经释放后的内存等；[请参考](https://www.jianshu.com/p/3a2df9b7c353?u_atoken=624ee094-c822-41f4-bcf6-5fc911b949b4&u_asession=01eVG2pb4kJntfS3zKvUwzqGfYyETO3tabyEQnuTAon_TnN22igfaidwsWELY9RifNX0KNBwm7Lovlpxjd_P_q4JsKWYrT3W_NKPr8w6oU7K-MRgJtdMBnfFCSLSUVOnOVslvTX-jMTLEIhdGFg3rxgWBkFo3NEHBv0PZUm6pbxQU&u_asig=05AIkmPveKLhKc57_OdCdsvt-tv1mPEVKqslVVGynFTbT9WaOj6qh6VtDAJm_nloRscdeiZLdACwRJP0Q-_KtIWYLXchKI6GH2gb5mlvFPUZXow5jCCY0e8BfJEho11gU03JeCwBFpFMcsZg05KiIAHH38pmWare3MAK3EHTudCgD9JS7q8ZD7Xtz2Ly-b0kmuyAKRFSVJkkdwVUnyHAIJzYWNnakQHfzMsbRrngPrsy9NKsbBmrNSrWzU3uuj5WqD6FPw117USKdEPc8n7HkzU-3h9VXwMyh6PgyDIVSG1W8WUHHzGsMGCPMAm4mVxK03yE2VADWQzXzi6j2h-w46rVswqjVBUe5JdYdXcEu8M8CJVuLapi3J-Jnbt5oFDaOJmWspDxyAEEo4kbsryBKb9Q&u_aref=mjjabe%2F7YflTFd2Ai7YF3fiowPQ%3D)
- **LeakSanitizer**：检测内存泄露，例如未释放的内存；
- **MemorySanitizer**：检测未初始化的内存，例如stack或者heap等在写入之前就使用。

Sanitizers目前已经默认加到了GCC和LLVM中，因此无需额外安装。

```sh
clang/gcc/g++ -fsanitize=address ... 
# -fsanitize=undefined
```


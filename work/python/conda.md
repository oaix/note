### 1 [复制conda环境](https://www.anaconda.com/blog/moving-conda-environments)

```sh
# old computer
conda env export > environment.yml
# new computer
conda env create -f environment.yml
```


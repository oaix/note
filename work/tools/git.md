# Git

## Problems

+ [一个本地git repository关联多个remote repository](https://www.zhihu.com/question/46543115)  
格式为：
`git push [远程库名] [本地分支名] : [远程分支名]`  
如果本地和远程的分支名一样，那么直接push:
```
git push github branch1
git push coding branch1
```
如果本地分支名和远程不同，则需要写全：
```
git push coding localbranch:branch1
git push github localbranch:branch1
```
如果有多个远程仓库，可以写一个push的脚本文件。

+ git撤销  
  - 撤销git add的内容, 取消暂存的文件  
  ```
  git reset HEAD file
  git rm --cached <file>
  ```
  - 提交完了才发现漏掉了几个文件没有添加，或者提交信息写错了  
  ```
  git commit -m 'initial commit'
  git add forgotten_file
  git commit --amend
  ```
  - 改乱了工作区某个文件的内容，想直接丢弃工作区的修改，用命令`git checkout -- file`  
  - git commit 版本回退  
  HEAD指向的版本就是当前版本，因此，Git允许我们在版本的历史之间穿梭，使用命令`git reset --hard commit_id`，穿梭前，用git log可以查看提交历史，以便确定要回退到哪个版本，要重返未来，用git reflog查看命令历史，以便确定要回到未来的哪个版本。
  - 本地误删文件恢复
  ```
  git checkout -- test.txt
  ```

+ git发布  
生成tag:
```
git checkout -b release develop // 新建并切换到release分支
git checkout master
git merge --no-ff release
git tag -a v1.2.0
git branch -d release
git push origin --tags
```

+ 分支合并  
切换分支：git checkout <name> --no-ff  
合并某分支到当前分支：git merge <name>  
删除分支：git branch -d <name>  
合并分支时，加上--no-ff参数就可以用普通模式合并，合并后的历史有分支，能看出来曾经做过合并，而fast forward合并就看不出来曾经做过合并。

+ 查看合并图  
用带参数的git log也可以看到分支的合并情况：
`git log --graph --pretty=oneline --abbrev-commit`

+ [git clone仓库的一部分是另外一个仓库](https://stackoverflow.com/questions/3796927/how-to-git-clone-including-submodules)  
git clone --recurse-submodules -j8 git://github.com/foo/bar.git
With version 1.9 of Git up until version 2.12 (-j flag only available in version 2.8+):
`git clone --recursive -j8 git://github.com/foo/bar.git`  
也可以先克隆仓库，然后获取子模块的仓库
```
git submodule init
git submodule update
```

+ config git hook  
下载[git.tar](git.tar)
```
git config --global init.templatedir ~/.git/gittemple
```
`git init`时会自动创建包含`commit-msg `,  `pre-commit`的hooks, 在commit时,会自动**非覆盖**添加` CHANGELOG,  .clang-format ,  .gitignore , LICENSE  README.md`文件



+ Init the commit template
  cp gitmessage.txt ~/.gitmessage.txt
  git config --global commit.template ~/.gitmessage.txt
  不建议使用hook
```sh
sudo apt-get install clang-format
cp clang-format your_git_project/.clang-format
cp commit-msg your_git_project/.git/hooks/
cp pre-commit your_git_project/.git/hooks/
```

+ 取消某项设置
  `git config --global --unset user.name`

+ [git tag](https://www.liaoxuefeng.com/wiki/0013739516305929606dd18361248578c67b8067c8c017b000/001376951885068a0ac7d81c3a64912b35a59b58a1d926b000)
  命令git tag <tagname>用于新建一个标签，默认为HEAD，也可以指定一个commit id；
  命令git tag -a <tagname> -m "blablabla..."可以指定标签信息；
  命令git tag可以查看所有标签。
  命令git push origin <tagname>可以推送一个本地标签；
  命令git push origin --tags可以推送全部未推送过的本地标签；
  命令git tag -d <tagname>可以删除一个本地标签；
  命令git push origin :refs/tags/<tagname>可以删除一个远程标签。

+ [git patch](https://juejin.im/post/5b5851976fb9a04f844ad0f4)
+ [git apply、git am打补丁.diff 和 .patch - 简书](https://www.jianshu.com/p/e5d801b936b6)
  [定位和解决git am冲突的方法 - 一程山水一程歌 - CSDN博客](https://blog.csdn.net/Qidi_Huang/article/details/61920472)
```sh
  # -1是指从当前id开始，向下提交次数，包含此次且计数从1开始。
  git format-patch [commit-id] -n
  git am *.patch
  # 如果出现冲突，那么先合并没有冲突的部分，然后收到更改冲突的部分
  git apply --rejected *.patch #生成*.rej文件
  git add 手动修改的文件
  git am --resolved
```


  + [git cherry-pick](https://blog.csdn.net/w958796636/article/details/78492017)
      branch1开发，进行多个提交，这是切换到branch2，想把之前branch1分支提交的commit都【复制】过来:
        单个commit只需要git cherry-pick commitid
        多个commit 只需要git cherry-pick commitid1..commitid100 #中间２个点
        **注意**，不包含第一个commitid ， 即  git cherry-pick (commitid1..commitid100]
        git cherry-pick <start-commit-id>..<end-commit-id>
        其中，<start-commit-id>到<end-commit-id>只需要commit-id的前6位即可，并且<start-commit-id>在时间上必须早于<end-commit-id>
       commitid1在时间上最早，即在提交的时间轴上依次为commitid1, commitid2 ... commitid100
       当合并发生冲突时，手动删除后，[git cherry-pick 使用指南 - 简书](https://www.jianshu.com/p/08c3f1804b36)
  ```sh
  git add .
  git commit -C <commitid> #使用以前的commit信息
  git cherry-pick --continue
  ```

+ 当使用`git commit --amend`造成本地和远程相同的commitid，不同的时间，使得不能合并
  [Git refusing to merge unrelated histories on rebase - Stack Overflow](https://stackoverflow.com/questions/37937984/git-refusing-to-merge-unrelated-histories-on-rebase)
  `git pull origin master --allow-unrelated-histories` 

+ [添加submodule](https://git-scm.com/book/zh/v2/Git-%E5%B7%A5%E5%85%B7-%E5%AD%90%E6%A8%A1%E5%9D%97)
  我们首先将一个已存在的 Git 仓库添加为正在工作的仓库的子模块。 你可以通过在 git submodule add 命令后面加上想要跟踪的项目 URL 来添加新的子模块。 在本例中，我们将会添加一个名为 “DbConnector” 的库。
  ```
  git submodule add https://github.com/chaconinc/DbConnector
  ```


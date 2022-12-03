[TOC]

## extensions

### c++

+ [vscode-cpptools](https://github.com/microsoft/vscode-cpptools)

  C/C++ IntelliSense, debugging, and code browsing. 在`settings.json`中添加如下设置:

  ```json
  	"C_Cpp.autoAddFileAssociations": true,
      "C_Cpp.enhancedColorization": "Enabled",
      "C_Cpp.default.includePath": ["${workspaceFolder}/include",
                  "${workspaceFolder}",
                  "/usr/include/eigen3",
                  "/usr/include/c++/5",
                  "/usr/include/pcl-1.7",
                  "/opt/ros/kinetic/include/opencv-3.3.1-dev",
                  "/opt/ros/kinetic/include"],
      "C_Cpp.default.intelliSenseMode":"gcc-x64",
      "C_Cpp.default.compilerPath": "/usr/bin/gcc",
      "C_Cpp.default.cStandard":"c11",
      "C_Cpp.default.cppStandard": "c++11",
      "C_Cpp.default.browse.limitSymbolsToIncludedHeaders": true,
      "C_Cpp.updateChannel": "Insiders",
  ```

  特别是`C_Cpp.default.includePath`配置的全局的`include path`，这样就不用每个c++工程都要配置`c_cpp_properties.json`。

+ [Clang-Format](http://clang.llvm.org/docs/ClangFormat.html)

  格式化`C/C++/Java/JavaScript/Objective-C/Protobuf/C#`代码，除了vscode需要安装`clang-format`插件外，还需要在系统中安装`clang-format`。该插件只是调用系统中`clang-format`.

  - 终端中安装`clang-format`

  ```sh
  sudo apt-get install clang-format
  ```

  - vscode clang-format配置

    ```json
    	"clang-format.executable": "/usr/bin/clang-format-6.0",
        "clang-format.fallbackStyle": "Google",
        "clang-format.assumeFilename": "/home/robosense/.vscode/.clang-format",
        "[cpp]": {
            "editor.defaultFormatter": "xaver.clang-format"
        },
    ```

    `clang-format.executable`: 可执行clang-format的路径

    `clang-format.assumeFilename`: 用于格式化的配置文件，将其放在home目录下的`.vscode`，或者其他位置均可。以下是配置文件`.clang-format`:

    ```python
    ---
    BasedOnStyle:  Google
    AccessModifierOffset: -2
    ConstructorInitializerIndentWidth: 2
    AlignEscapedNewlinesLeft: false
    AlignTrailingComments: true
    AllowAllParametersOfDeclarationOnNextLine: false
    AllowShortIfStatementsOnASingleLine: false
    AllowShortLoopsOnASingleLine: false
    AllowShortFunctionsOnASingleLine: None
    AllowShortLoopsOnASingleLine: false
    AlwaysBreakTemplateDeclarations: true
    AlwaysBreakBeforeMultilineStrings: false
    BreakBeforeBinaryOperators: false
    BreakBeforeTernaryOperators: false
    BreakConstructorInitializersBeforeComma: true
    BinPackParameters: true
    ColumnLimit:    120
    ConstructorInitializerAllOnOneLineOrOnePerLine: true
    DerivePointerBinding: false
    PointerBindsToType: true
    ExperimentalAutoDetectBinPacking: false
    IndentCaseLabels: true
    FixNamespaceComments: true
    MaxEmptyLinesToKeep: 1
    NamespaceIndentation: None
    ObjCSpaceBeforeProtocolList: true
    PenaltyBreakBeforeFirstCallParameter: 19
    PenaltyBreakComment: 60
    PenaltyBreakString: 1
    PenaltyBreakFirstLessLess: 1000
    PenaltyExcessCharacter: 1000
    PenaltyReturnTypeOnItsOwnLine: 90
    SpacesBeforeTrailingComments: 2
    Cpp11BracedListStyle: false
    Standard:        Auto
    IndentWidth:     2
    TabWidth:        2
    UseTab:          Never
    IndentFunctionDeclarationAfterType: false
    SpacesInParentheses: false
    SpacesInAngles:  false
    SpaceInEmptyParentheses: false
    SpacesInCStyleCastParentheses: false
    # if else for 后的空格
    SpaceAfterControlStatementKeyword: true
    SpaceBeforeAssignmentOperators: true
    ContinuationIndentWidth: 4
    SortIncludes: false
    SpaceAfterCStyleCast: false
    
    # Configure each individual brace in BraceWrapping
    BreakBeforeBraces: Custom
    
    # Control of individual brace wrapping cases
    BraceWrapping: {
        AfterClass: 'true'
        AfterControlStatement: 'true'
        AfterEnum : 'true'
        AfterFunction : 'true'
        AfterNamespace : 'true'
        AfterStruct : 'true'
        AfterUnion : 'true'
        BeforeCatch : 'true'
        BeforeElse : 'true'
        IndentBraces : 'false'
    }
    ...
    ```

  - 格式化快捷键

    linux下默认为`ctrl+shift+i`，可自行更改快捷键。

+ CMake

  CMake language support for Visual Studio Code

+ CMake Tools

+ [cmake-format](https://marketplace.visualstudio.com/items?itemName=cheshirekow.cmake-format)

  给CMakeLists.txt格式化，除了安装该插件外，系统需要安装`cmake-format`中:

  ```sh
  sudo apt-get install -y python3-pip
  sudo -H pip install --upgrade pip
  sudo -H pip3 install cmake_format
  ```

  - 配置

    `settings.json`中配置如下：

    ```json
        "cmakeFormat.args": [
            "--config-file", "/home/robosense/.vscode/.cmake-format.py"
        ],
        "cmakeFormat.exePath": "/usr/local/bin/cmake-format", 
    ```

    其中，`cmakeFormat.exePath`为`cmake-format`可执行软件的位置，`cmakeFormat.args`为格式化配置文件`.cmake-format.py`，以下为`.cmake-format.py`的具体内容:

    ```python
    # cmake-format configuration file
    # Use run-cmake-format.py to reformat all cmake files in the source tree
    # How wide to allow formatted cmake files
    line_width = 90
    
    # How many spaces to tab for indent
    tab_size = 2
    
    # If arglists are longer than this, break them always
    max_subargs_per_line = 2
    
    # If a positional argument group contains more than this many arguments, then
    # force it to a vertical layout.
    max_pargs_hwrap = 6
    
    # If true, separate flow control names from their parentheses with a space
    separate_ctrl_name_with_space = False
    
    # If true, separate function names from parentheses with a space
    separate_fn_name_with_space = False
    
    # If a statement is wrapped to more than one line, than dangle the closing
    # parenthesis on it's own line
    dangle_parens = False
    
    # If the trailing parenthesis must be 'dangled' on its on line, then align it to
    # this reference: `prefix`: the start of the statement,  `prefix-indent`: the
    # start of the statement, plus one indentation  level, `child`: align to the
    # column of the arguments
    dangle_align = 'prefix'
    
    # If the statement spelling length (including space and parenthesis) is smaller
    # than this amount, then force reject nested layouts.
    min_prefix_chars = 4
    
    # If the statement spelling length (including space and parenthesis) is larger
    # than the tab width by more than this amount, then force reject un-nested
    # layouts.
    max_prefix_chars = 10
    
    # If a candidate layout is wrapped horizontally but it exceeds this many lines,
    # then reject the layout.
    max_lines_hwrap = 2
    
    # What style line endings to use in the output.
    line_ending = 'unix'
    
    # Format command names consistently as 'lower' or 'upper' case
    command_case = 'lower'
    
    # Format keywords consistently as 'unchanged' 'lower' or 'upper' case
    keyword_case = 'unchanged'
    
    # enable comment markup parsing and reflow
    enable_markup = True
    
    # If true, the argument lists which are known to be sortable will be sorted
    # lexicographicall
    enable_sort = False
    
    # If comment markup is enabled, don't reflow the first comment block in
    # eachlistfile. Use this to preserve formatting of your
    # copyright/licensestatements.
    first_comment_is_literal = False
    
    # If comment markup is enabled, don't reflow any comment block which matchesthis
    # (regex) pattern. Default is `None` (disabled).
    literal_comment_pattern = None
    
    # Specify structure for custom cmake functions
    additional_commands = {
      "foo": {
        "flags": [
          "BAR",
          "BAZ"
        ],
        "kwargs": {
          "HEADERS": "*",
          "DEPENDS": "*",
          "SOURCES": "*"
        }
      }
    }
    ```

  - 快捷键

    `ctrl+shift+i`

+ TODO Highlight

  配置：

  ```json
      "todohighlight.isEnable": true,
      "todohighlight.isCaseSensitive": true,
  ```


### [Settings Sync](https://www.cnblogs.com/lychee/p/11214032.html)

使用同步插件，保存所有的设置以及extensions。后续在新的机器上只需要安装此插件，然后同步以前的配置即可。



## vscode for ros

https://erdalpekel.de/?p=157

https://medium.com/@tahsincankose/a-decent-integration-of-vscode-to-ros-4c1d951c982a

https://blog.csdn.net/Kalenee/article/details/103828448
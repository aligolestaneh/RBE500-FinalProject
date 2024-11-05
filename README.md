# RBE500-FinalProject

# Setup 
1. First Setup the Github , if you are using the git on your system for the first time, follow these links [1. Generate SSH Key for here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent), [2. Add to your github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

2. Clone git repo in your ROS workspace
    ```
    $ cd <your-workspace>/src
    $ git clone git@github.com:aligolestaneh/RBE500-FinalProject.git
    $ cd <your-workspace> 
    $ rosdep install --from-paths src --ignore-src -r -y
    $ colcon build --symlink-install --packages-select rbe500_final_project
    ```

# Running Nodes
TODO:

# Writing Test Cases 
## CMakelist Edits
All test cases has been written up in the test folder. You may add your test cases aswell by creating your own test `cpp` file or editing the existing ones. Please don't edit `main.cpp`. Follow these steps if you create your own test files, then make sure you add it into `CMakelist.txt`
```
set(TESTFILES
  test/main.cpp
  test/simple_test.cpp
  test/<your_test_file_name>_test.cpp) #<<------ add here

```
## Compiling and Test case results

In order to view the test cases results and you need to build the package first:
```
$ cd <your-workspace> 
$ colcon build --symlink-install --packages-select rbe500_final_project
```
Now you can build the test cases:
1. Compiling without test cases results:
    ```
    $ colcon test --packages-select rbe500_final_project
    ```
2. Compiling the test cases along with result verbose:
    ```
    colcon test --packages-select rbe500_final_project --event-handler=console_direct+
    ```

If you need to check the results after building:
```
$ colcon test --packages-select rbe500_final_project
```


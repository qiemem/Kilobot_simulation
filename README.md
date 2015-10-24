Kilobot Simulator
---

### Requirements

- glew
- freeglut
- X11

### Installation

#### OS X

1. Install [XQuartz](http://xquartz.macosforge.org/) (though there's a good chance you already have it).

2. Install glew and freeglut. If you use [Homebrew](http://brew.sh/) (which you really should), this is just:
  
  ```
  brew install glew
  brew install homebrew/x11/freeglut
  ```
  
  I would guess that MacPorts and Fink also have these libraries.

3. Compile and run!
  
  ```
  make
  ./kilobot
  ```

Note that if your libraries are installed to somewhere besides `/usr/local`, you'll have to edit the Makefile to point to the right directories. Just change `/usr/local/lib` and `/usr/local/include` to point to wherever you put such things.

#### Linux

I'm afraid you'll have to edit the Makefile. You should be able to use your package manager (e.g. apt, yum, etc) to install glew and freeglut. Edit the Makefile to set `INCLUDE_PATHS` and `LIBRARY_PATHS` to point to where your libraries are installed. (I think) Remove `-framework OpenGL` from the `LINKER_FLAGS`.

I would happily accept any PRs (or tips) on how to make this more portable.



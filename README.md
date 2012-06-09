# openni-overtone

OpenNI with Overtone, designed for a modern dance performance.

## Requirements

* Kinect with USB adapter

## Usage

* Setup the Kinect. Follow the [instructions](http://code.google.com/p/simple-openni/wiki/Installation) for SimpleOpenNI. 
* Test [SimpleOpenNI](http://code.google.com/p/simple-openni/) using Processing. If this fails, test OpenNI and NITE without Processing.
* Install [Leiningen 2](https://github.com/technomancy/leiningen/wiki/Upgrading). I am using Leiningen 2 for future OpenGL implementations (quil requires Leiningen 2 for OpenGL support). Replace lein with lein2 below if necessary.
* Clone this project
* `cd openni-overtone`
* lein deps
* Make sure the SimpleOpenNI native library is on your java classpath. The easiest way is to put the SimpleOpenNI JNI's into openni-overtone/native in the right location for your OS.
* connect the Kinect
* `lein run -m openni-overtone.core`

You probably want to setup Emacs for Overtone using [Emacs Live](https://github.com/overtone/emacs-live). For lein2, instead of lein plugins, edit `~/.lein/project.clj` as in [documentation](https://github.com/technomancy/leiningen/wiki/Upgrading).

## License

Copyright (C) 2012 Charles Dietrich.

Distributed under the GNU General Public License, v3. See LICENSE for full text.

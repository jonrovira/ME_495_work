# Master of Science in Robotics Site



## Jekyll Overview

### Built with Jekyll
Jekyll is a simple, blog-aware, static site generator. It takes a template directory containing raw text files in various formats, runs it through Markdown and Liquid converters, and spits out a complete, ready-to-publish static website suitable for serving with your favorite web server. Jekyll also happens to be the engine behind GitHub Pages, which means you can use Jekyll to host your project's page, blog, or website from GitHub's servers for free (taken from Jekyll's website: http://jekyllrb.com/docs/home/).

### Get your workstation set up
* Install <a href="https://www.ruby-lang.org/en/downloads/>Ruby</a>
* Install <a href="rubygems.org/pages/download">RubyGems</a>
* Install <a href="nodejs.org">NodeJS</a>
* Install <a href="jekyllrb.com/docs/installation">Jekyll</a>

### Basic Usage (recommended)
In one terminal, build the jekyll site, watching for any changes (run in site root directory)
```
$  jekyll build --watch
```
In another terminal, start a local server (run in site root directory)
```
$  jekyll serve
```
View the site in your browser at
```
localhost:4000/msr-site/
```



## File structure (as of 9/17/14)
```
|-- README.md (this)
|-- _config.yml (overall configuration file for the site)
|-- _includes (all the markup partials)
|   |-- footer.html
|   |-- head.html
|   |-- header.html
|-- _layouts (page markup templates)
|   |-- default.html
|   |-- post.html
|   |-- project.html
|-- _projects (markdown files that make up the "projects" jekyll collection)
|   |-- baxter-in-gazebo.md
|   |-- pick-and-place-demo.md
|   |-- robot-web-tools-with-baxter.md
|-- _resources (markdown files that make up the "resources" jekyll collection)
|   |-- baxter.md
|   |-- rtt.md
|-- _site (the entire site after it is processed by Jekyll)
|   |-- README.md
|   |-- index.html
|   |-- projects
|   |-- public
|   |-- resources
|   |-- students
|-- _students (markdown files that make up the "students" jekyll collection)
|   |-- 2011 (class of 2011)
|   |   |-- todd.md
|   |-- 2012
|   |   |-- jarvis.md
|   |-- 2013
|   |   |-- jon.md
|   |-- 2014
|   |   |-- kevin.md
|-- index.html (home page of the site)
|-- projects.html (projects landing page of the site)
|-- public (static content including fonts, images, js, and css files)
|   |-- fonts
|   |-- images
|   |-- javascripts
|   |-- stylesheets
|-- resources.html (resources landing page of the site)
|-- students.html (students landing page of the site)
```



## More on how Jekyll works

### The Jekyll Engine
First, if you look inside the \_site directory, you'll see that no directories or files there begin with an underscore (\_). The contents of that directory are the end result of Jekyll's processing engine. All of the files and directories in the root directory of the repository that do begin with an underscore, on the other hand, are "raw". They either include markup that will be included within pages of the final site or they contain markdown and "Front Matter" (which I'll explain later) that will be converted into markup by Jekyll's engine. One of the two commands that you need to run in order to host the site on a local server:
```
jekyll build --watch
``` 
runs that engine, processing and reprocessing the "raw" files every time you make a change to a file. The files and directories in the root directory of the repository that _don't_ begin with an underscore are ignored by Jekyll and will remain the exact same in the _site directory.

### Front Matter
Any file that contains a YAML front matter block will be processed by Jekyll as a special file. The front matter must be the first thing in the file and must take the form of valid YAML, set between triple-dashed lines (taken from Jekyll's documentation: http://jekyllrb.com/docs/frontmatter/). Here's a basic example that you'll find in the index.html file:
```
---
layout: default
title: Home
---
```
This first item tells Jekyll to take all of the markup in index.html and plug it into the _layouts/default.html template to take the place of the {{ content }} variable found in that template file.
The second item tells Jekyll to create a variable, page.title, that you can use in the markup of the template. For example, in _layouts/default.html, you could write:
```
<head>
	<title>{{ page.title }}</title>
</head>
```
and that would render as:
```
<head>
	<title>Home</title>
</head>
```

### Collections
Collections allow you to define a new type of document that can be somewhat conceptualized as an object type, each having its own unique properties and namespaces. These collections are declared in the _config.yml file:
```
collections:
  projects:
    output: true
    permalink: /projects/:path/
  students:
    output: false
    years: [2011, 2012, 2013, 2014]
  resources:
    output: true
    permalink: /resources/:path/
    tags: [navigation, manipulation, vision, ...
```
For this site, we use three collections: projects, students, and resources, the contents of which can each be found in the corresponding directores: _projects, _students, and _resources. Notice that each of these directories begins with an underscore. This is because each file in those directories only contains some combination of markdown and front-matter. Let's look at _students/2013/jon.md as an example:
```
---
name:       Jon Rovira
first_name: Jon
last_name:  Rovira
class_year: 2013
focus:      neuroscience, robotics
website:    http://google.com
summary:    Lorem ipsum dolor sit amet, an qui...
---
```
This file represents a student in the students collection and only contains YAML front matter. You can see how powerful collections are if we take a look at a snippet of students.html:
```
<section id="students-list">
	<ul>
		{% for student in site.students %}
			<li id="{{ student.first_name }}-{{ student.last_name }}" class="class-year-{{ student.class_year }}">
				<img src="http://unsplash.it/500?random">
				<div class="student-info">
					<h2 class="student-name">{{ student.name }}</h2>
					<h3 class="student-year">Class of {{ student.class_year }}</h3>
					<h3 class="student-focuses">Focuses: {{ student.focus }}</h3>
					<h3 class="student-website">Website: <a href="{{ student.website }}">{{ student.website }}</a></h3>
					<p class="student-summary">{{ student.summary }}</p>
				</div>
			</li>
		{% endfor %}
	</ul>
</section>
```
The ```{% %}``` tags represent liquid syntax and their contents are processed by Jekyll to render static HTML in the final site. You can see that all of the students in the students collection can be referenced with ```site.students``` and iterated through with a for loop. In this specific for loop, for each student in the students collection, we pull their name, class year, academic focus, website address, and summary using ```{{ }}``` tags. All of a particular student's information is defined in their markdown file just like Jon's which we saw above. You can find more useful information about collections in Jekyll's website (http://jekyllrb.com/docs/collections/).



## Maintaining the Site

### Hosting
For now, the MSR site is  located at http://jonrovira.github.io/msr-site/ and hosted on Github Pages, using the gh-pages branch of this repository. Whenever you want to make a change to the site, just push the edits to that branch. Notice the base URL, "/msr-site", which will always be the same as the repository name. This is important and has to be specified in the _config.yml file with the line:
```
baseurl: /msr-site
```
All anchor tags that point to an address within the site need those href addresses to be prefixed with that base URL, for example in line 11 of _includes/header.html:
```
<li><a href="{{ site.baseurl }}/">HOME</a></li>
```

### Adding Projects
To add a project, just create a .md file in the _projects directory with front matter at the beginning that follows this format (taken from baxter-in-gazebo.md):
```
---
layout:             project
image:              /baxter-in-gazebo2.png
title:              Baxter in Gazebo
author:             Jarvis Schultz
date:               August 29th, 2014
demo:               https://github.com/jonrovira/ME_495_work
requirements:       [python-wstool,
                    python-rosdep,
                    ros-hydro-pcl-conversions,
                    ros-hydro-control-msgs,
                    ros-hydro-cmake-modules,
                    ros-hydro-qt-build,
                    ros-hydro-moveit-full,
                    ros-hydro-driver-common,
                    ros-hydro-image-common,
                    ros-hydro-rostest-gazebo,
                    Baxter Simulation repository (must request access by emailing RSDK.support@rethinkrobotics.com]
overview:           Learn how to quickly get a Baxter simulation running in Gazebo.
                    Execute the joint position keyboard example in the simulation.
---
```
Following that front matter, just add content in normal markdown.

### Adding Resources
Adding a resource is simply a matter of creating a file in the _resources directory with the file name in the following format:
```
YEAR-MONTH-DAY-title.md
```
You will also need to start that file off with a little bit of front matter in this format (taken from 2014-09-17-rtt.md):
```
---
title:   Rapidly Exploring Random Tree (RTT)
author:  John Smith
date:    August 8th, 2014
tags:    [algorithms, probabilistic data structures, search]
---
```
After the front matter, just type the content in whichever format you think is appropriate using markdown.

In addition to creating the markdown file, you will need to check the _config.yml file under collections -> resources -> tags. The line will be a list of tags that looks like this:
```
tags: [navigation, manipulation, vision, perception... ]
```
If the resource you're adding contains tags that are not currently in the list in _config.yml, add those items at the end of the list. The site will automatically alphabetize them.

### Adding Students
In order to add a student, create a .md file with the title being his/her name and place it in the subdirectory of _students that corresponds to his/her class year. For example, according to his markdown file, Jon Rovira will graduate in 2013, so the file should be placed in _students/2013/. The markdown file will only include front matter and should take on the following format:
```
---
name:       Jon Rovira
first_name: Jon
last_name:  Rovira
class_year: 2013
focus:      neuroscience, robotics
website:    http://google.com
summary:    Lorem ipsum dolor sit amet, an...
---
```









#!/usr/bin/env python

# Release lvgl, lv_examples, lv_drivers. docs, blog and prepare the development of the next major, minoror bugfix release
# Usage: ./release,py bugfix | minor | major
# The option means what type of versin to prepare for development after release
#
# STEPS:
#  - clone all 5 repos
#  - get the version numnber from lvgl.h
#  - set release branch (e.g. "release/v7")
#  - prepare lvgl
#    - run lv_conf_internal.py
#    - run code formatter 
#    - clear LVGL_VERSION_INFO (set to "")
#    - run Doxygen 
#    - update the version in lvgl's library.json, library.properties, lv_conf_template.h
#    - update CHANGELOG.md
#    - commit changes 
#  - prepare lv_examples
#    - upadte the required LVGL version in lv_examples.h (LV_VERSION_CHECK) 
#    - update the version in lv_ex_conf_template.h
#  - prepare lv_drivers
#    - update the version in library.json, lv_drv_conf_template.h      
#  - prepare docs
#    - update API XML 
#    - clear the versiopn info (should be plain vx.y.z)
#  - tag all repos with the new version
#  - merge to release branches
#  - blog: add release post
#  - push tags and commits
#  - docs: run ./updade.py release/vX
#
# If --patch
#  - merge master to dev branches
#  - increment patch version by 1 and append "-dev". E.g. "vX.Y.(Z+1)-dev"
#  - update version numbers in lvgl and docs
#  - commit and push
#  - docs: run ./updade.py latest dev
#
# Else (not --patch)
#  - merge master to dev  
#  - merge the dev to master
#  - increment version number like "vX.(Y+1).0-dev"
#  - apply the new version in dev branches of lvgl, lv_examples, lv_drivers, docs
#  - commit and push to dev branches
#  - docs: run ./updade.py latest dev

import re
import os, fnmatch
import os.path
from os import path
from datetime import date
import sys

upstream_org_url = "https://github.com/lvgl/"
workdir = "./release_tmp"

ver_major = -1
ver_minor = -1
ver_patch = -1

ver_str = ""
dev_ver_str = ""
release_br = ""
release_note = ""

prepare_type = ['major', 'minor', 'bugfix']

dev_prepare = 'minor'

def upstream(repo):
    return upstream_org_url + repo + ".git"

def cmd(c, exit_on_err = True):
  print("\n" + c)
  r = os.system(c)       
  if r:
    print("### Error: " + str(r))
    if exit_on_err: exit(int(r))
    
def define_set(fn, name, value):    
    print("In " + fn + " set " + name + " to " + value)
    
    new_content = ""
    f = open(fn, "r")
      
    for i in f.read().splitlines():
        r = re.search(r'^ *# *define +' + name, i)
        if r: 
            d = i.split("define")
            i = d[0] + "define " + name + " " + value 
        new_content += i + '\n'
     
    f.close()
    
    f = open(fn, "w")
    f.write(new_content)
    f.close()
    
def clone_repos():
    cmd("rm -fr " + workdir)
    cmd("mkdir " + workdir)
    os.chdir(workdir)

    #For debuging just copy the repos
    #cmd("cp -a ../repos/. .")
    #return

    cmd("git clone " + upstream("lvgl") + " lvgl; cd lvgl; git checkout master")
    cmd("git clone " + upstream("lv_examples") + "; cd lv_examples; git checkout master")
    cmd("git clone " + upstream("lv_drivers") + "; cd lv_drivers; git checkout master")
    cmd("git clone --recurse-submodules " + upstream("docs") + "; cd docs; git checkout master")
    cmd("git clone " + upstream("blog") + "; cd lv_drivers; git checkout blog")

def get_lvgl_version(br):
    print("Get LVGL's version")
    
    global ver_str, ver_major, ver_minor, ver_patch, release_br 
    
    os.chdir("./lvgl")
    
    cmd("git checkout " + br)
    
    f = open("./lvgl.h", "r")
      
    lastNum = re.compile(r'(?:[^\d]*(\d+)[^\d]*)+')
    for i in f.read().splitlines():
        r = re.search(r'^#define LVGL_VERSION_MAJOR ', i)
        if r: 
            m = lastNum.search(i)
            if m: ver_major = m.group(1)

        r = re.search(r'^#define LVGL_VERSION_MINOR ', i)
        if r: 
            m = lastNum.search(i)
            if m: ver_minor = m.group(1)
      
        r = re.search(r'^#define LVGL_VERSION_PATCH ', i)
        if r: 
            m = lastNum.search(i)
            if m: ver_patch = m.group(1)
     
    f.close()
    
    cmd("git checkout master")
    
    ver_str =  "v" + str(ver_major) + "." + str(ver_minor) + "." + str(ver_patch)
    print("New version:" + ver_str)
    
    release_br = "release/v" + ver_major
    
    os.chdir("../")
    
def update_version():
    templ = fnmatch.filter(os.listdir('.'), '*templ*')
    
    if templ[0]:    
        print("Updating version in " + templ[0])
        cmd("sed -i -r 's/v[0-9]+\.[0-9]+\.[0-9]+/"+ ver_str +"/' " + templ[0])
    
    if os.path.exists("library.json"):  
        print("Updating version in library.json")
        cmd("sed -i -r 's/[0-9]+\.[0-9]+\.[0-9]+/"+ ver_str[1:] +"/' library.json")
        
    if path.exists("library.properties"):  
        print("Updating version in library.properties")
        cmd("sed -i -r 's/version=[0-9]+\.[0-9]+\.[0-9]+/"+ "version=" + ver_str[1:] + "/' library.properties")
    
def lvgl_prepare():
    print("Prepare lvgl")
    
    global ver_str, ver_major, ver_minor, ver_patch 
    
    os.chdir("./lvgl")
    define_set("./lvgl.h", "LVGL_VERSION_INFO", '\"\"')
    
    # Run some scripts
    os.chdir("./scripts")
    cmd("./code-format.sh")
    cmd("./lv_conf_checker.py")
    cmd("doxygen")
    os.chdir("../")

    update_version()

    #update CHANGLELOG
    new_content = ""
    f = open("./CHANGELOG.md", "r")
    
    global release_note
    release_note = ""
    note_state = 0  
    for i in f.read().splitlines():
        if note_state == 0:
            r = re.search(r'^## ' + ver_str, i)
            if r: 
                i = i.replace("planned on ", "") 
                note_state+=1
                
        elif note_state == 1: 
            r = re.search(r'^## ', i)
            if r:
                note_state+=1
            else:
                release_note += i + '\n'
                    
        new_content += i + '\n'
     
    f.close()
    
    f = open("./CHANGELOG.md", "w")
    f.write(new_content)
    f.close()
    
    cmd('git commit -am "prepare to release ' + ver_str + '"')
    
    os.chdir("../")
    
    
def lv_examples_prepare():
    print("Prepare lv_examples")
    global ver_str, ver_major, ver_minor, ver_patch 
    
    os.chdir("./lv_examples")
    
    update_version()
    
    cmd("sed -i -r 's/LV_VERSION_CHECK\([0-9]+, *[0-9]+, *[0-9]+\)/"+ "LV_VERSION_CHECK(" + ver_major + ", " + ver_minor + ", " + ver_patch + ")/' lv_examples.h")
    
    cmd('git commit -am "prepare to release ' + ver_str + '"')
    
    os.chdir("../")
    
def lv_drivers_prepare():
    print("Prepare lv_drivers")
    global ver_str, ver_major, ver_minor, ver_patch 
    
    os.chdir("./lv_drivers")
    
    update_version()
    
    cmd('git commit -am "prepare to release ' + ver_str + '"')
    
    os.chdir("../")
    
def docs_prepare():
    print("Prepare docs")
    global ver_str, ver_major, ver_minor, ver_patch 
    
    os.chdir("./docs")
    
    cmd("git co latest --") 
    cmd("rm -rf xml");  
    cmd("cp -r ../lvgl/docs/api_doc/xml .");  
    cmd("git add xml");
    
    cmd("sed -i -r \"s/'v[0-9]+\.[0-9]+\.[0-9]+.*'/\'" + ver_str + "'/\" conf.py")   
    
    cmd('git commit -am "prepare to release ' + ver_str + '"')
    
    os.chdir("../")
    
def blog_add_post():
    global ver_str, release_note
    
    os.chdir("./blog/_posts")
    
    post = "---\nlayout: post\ntitle: " + ver_str + " is released\nauthor: \"kisvegabor\"\ncover: /assets/release_cover.png\n---\n\n"
    post += release_note
    
    today = date.today()
    d = today.strftime("%Y-%m-%d")
    
    f = open(d + "_release_" + ver_str + ".md", "w")
    f.write(post)
    f.close()
    
    cmd("git add .")
    cmd("git commit -am 'Add " + ver_str + " release post'")
    
    os.chdir("../../")
        
def add_tags():
    global ver_str
    tag_cmd = " git tag -a " + ver_str + " -m 'Release " + ver_str + "' " 
    cmd("cd lvgl; " + tag_cmd)    
    cmd("cd lv_examples; " + tag_cmd)    
    cmd("cd lv_drivers; " + tag_cmd)    
    cmd("cd docs; " + tag_cmd)    
    
def update_release_branches():
    global release_br
    merge_cmd = " git checkout " + release_br + "; git pull origin " + release_br + "; git merge master  -X ours; git push origin " + release_br + "; git checkout master" 
    cmd("cd lvgl; " + merge_cmd)    
    cmd("cd lv_examples; " + merge_cmd)    
    cmd("cd lv_drivers; " + merge_cmd)    
    
    merge_cmd = " git checkout " + release_br + "; git pull origin " + release_br  + "; git merge latest  -X ours; git push origin " + release_br + "; git checkout latest" 
    cmd("cd docs; " + merge_cmd)
    
def publish_master():   
    pub_cmd = "git push origin master; git push origin " + ver_str
    cmd("cd lvgl; " + pub_cmd)    
    cmd("cd lv_examples; " + pub_cmd)    
    cmd("cd lv_drivers; " + pub_cmd)    

    pub_cmd = "git push origin latest; git push origin " + ver_str
    cmd("cd docs; " + pub_cmd)    
    cmd("cd docs; git checkout master; ./update.py " + release_br)
    
    pub_cmd = "git push origin master"
    cmd("cd blog; " + pub_cmd)    
    
    
def merge_to_dev():
    merge_cmd = "git checkout dev; git merge master -X ours; git checkout master"
    cmd("cd lvgl; " + merge_cmd)    
    
    merge_cmd = "git checkout dev; git merge latest -X ours; git checkout master"
    cmd("cd docs; " + merge_cmd)    
    
    
def merge_from_dev():
    merge_cmd = "git checkout master; git merge dev;"
    cmd("cd lvgl; " + merge_cmd)    
    
    merge_cmd = "git checkout latest; git merge dev;"
    cmd("cd docs; " + merge_cmd)    
        

def lvgl_update_master_version():
    global ver_major, ver_minor, ver_patch, ver_str
    
    os.chdir("./lvgl")

    cmd("git checkout master")
    define_set("./lvgl.h", "LVGL_VERSION_MAJOR", ver_major)
    define_set("./lvgl.h", "LVGL_VERSION_MINOR", ver_minor)
    define_set("./lvgl.h", "LVGL_VERSION_PATCH", ver_patch)
    define_set("./lvgl.h", "LVGL_VERSION_INFO", "dev")
    
    templ = fnmatch.filter(os.listdir('.'), '*templ*')
    if templ[0]:    
        print("Updating version in " + templ[0])
        cmd("sed -i -r 's/v[0-9]+\.[0-9]+\.[0-9]+/"+ ver_str +"/' " + templ[0])
    
    
    cmd("git commit -am 'Update version'")
    
    os.chdir("../")

def docs_update_latest_version():
    global ver_str
    
    os.chdir("./docs")
    cmd("git checkout latest --")
    cmd("sed -i -r \"s/'v[0-9]+\.[0-9]+\.[0-9]+.*'/\'" + ver_str + "'/\" conf.py")   
    cmd("git commit -am 'Update version'")
    cmd("git checkout master --")
    
    os.chdir("../")
        
        
def lvgl_update_dev_version():
    global ver_major, ver_minor, ver_patch, dev_ver_str
    
    os.chdir("./lvgl")

    cmd("git checkout dev")
    define_set("./lvgl.h", "LVGL_VERSION_MAJOR", ver_major)
    define_set("./lvgl.h", "LVGL_VERSION_MINOR", ver_minor)
    define_set("./lvgl.h", "LVGL_VERSION_PATCH", ver_patch)
    define_set("./lvgl.h", "LVGL_VERSION_INFO", "\"dev\"")
    
    templ = fnmatch.filter(os.listdir('.'), '*templ*')
    if templ[0]:    
        print("Updating version in " + templ[0])
        cmd("sed -i -r 's/v[0-9]+\.[0-9]+\.[0-9]+/"+ dev_ver_str +"/' " + templ[0])
    
    
    cmd("git commit -am 'Update dev version'")
    cmd("git checkout master")
    
    os.chdir("../")

def docs_update_dev_version():
    global dev_ver_str
    
    os.chdir("./docs")
    cmd("git checkout dev --")
    cmd("sed -i -r \"s/'v[0-9]+\.[0-9]+\.[0-9]+.*'/\'" + dev_ver_str + "'/\" conf.py")   
    cmd("git commit -am 'Update dev version'")
    cmd("git checkout master --")
    
    os.chdir("../")


def publish_dev():   
    pub_cmd = "git checkout dev; git push origin dev"
    cmd("cd lvgl; " + pub_cmd)    

    pub_cmd = "git checkout dev; git push origin dev"
    cmd("cd docs; " + pub_cmd)    
    cmd("cd docs; git checkout master; ./update.py latest dev")

def cleanup():
    os.chdir("../")
    cmd("rm -fr " + workdir)

if __name__ == '__main__':
    if(len(sys.argv) != 2):
        print("Argument error. Usage ./release.py bugfix | minor | major") 
        exit(1)
        
    dev_prepare = sys.argv[1]
    if not (dev_prepare in prepare_type): 
        print("Invalid argument. Usage ./release.py bugfix | minor | major") 
        exit(1)
        
    clone_repos()
    get_lvgl_version("master")
    lvgl_prepare()
    lv_examples_prepare() 
    lv_drivers_prepare()
    docs_prepare()
    blog_add_post()
    add_tags()
    update_release_branches()
    publish_master()
 
    if dev_prepare == 'bugfix': 
        ver_patch = str(int(ver_patch) + 1)
        ver_str = "v" + ver_major + "." + ver_minor + "." + ver_patch + "-dev"    

        print("Prepare bugfix version " + ver_str)

        lvgl_update_master_version()
        docs_update_latest_version()

        get_lvgl_version("dev")
        dev_ver_str = "v" + ver_major + "." + ver_minor + "." + ver_patch + "-dev"
        merge_to_dev()
        
        lvgl_update_dev_version()
        docs_update_dev_version()
        publish_dev()
    else:
        get_lvgl_version("dev")
        if dev_prepare == 'minor': 
            ver_minor = str(int(ver_minor) + 1)
            ver_patch = "0"
        else:
            ver_major = str(int(ver_major) + 1)
            ver_minor = "0"
            ver_patch = "0"
                
        dev_ver_str = "v" + ver_major + "." + ver_minor + "." + ver_patch + "-dev"
        
        print("Prepare minor version " + ver_str)

        merge_to_dev()
        merge_from_dev()

        lvgl_update_dev_version()
        docs_update_dev_version()
        publish_dev()
        
    cleanup()
    

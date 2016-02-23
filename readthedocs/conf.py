import os
import subprocess

tagfiles = {}

# attempt to retrieve ompl tagfile
try:
   tagfile = '_build/ompl.tag'
   subprocess.check_call(['curl','--output',tagfile,'http://ompl.kavrakilab.org/core/ompl.tag'])
   tagfiles = {tagfile:'http://ompl.kavrakilab.org/'}
except subprocess.CalledProcessError as ex:
   print('got error {} when trying to connect to ompl website for tag file, ignoring ...'.format(ex))

def multisplit(s, *needles):
   ret = []
   for needle in needles:
      pre,s = s.split(needle,1)
      ret.append(pre)
      ret.append(needle)
   ret.append(s)
   return ret

# in dependency order
pkgs = ['pr_bgl','ompl_lemur','or_lemur','prpy_lemur']
for ipkg,pkg in enumerate(pkgs):
   
   # doxygen files
   fn_config = 'config.txt'
   fn_header = 'header.html'
   fn_footer = 'footer.html'
   fn_style = 'style.css'
   
   tagfile = '_build/html/{pkg}.tag'.format(**locals())
   str_tagfiles = ' '.join(['{}={}'.format(k,v) for k,v in tagfiles.items()])
   
   # doxygen config file
   fp = open(fn_config,'w')
   fp.write('''
PROJECT_NAME = "{pkg}"
INPUT = ../{pkg}
RECURSIVE = YES
GENERATE_LATEX = NO
GENERATE_HTML = YES
OUTPUT_DIRECTORY = _build/html/{pkg}
HTML_OUTPUT = .
HTML_HEADER = {fn_header}
HTML_FOOTER = {fn_footer}
HTML_STYLESHEET = {fn_style}
TAGFILES = {str_tagfiles}
EXTERNAL_GROUPS = NO
EXTERNAL_PAGES = NO
GENERATE_TAGFILE = {tagfile}
'''.format(**locals()))
   fp.close()
   
   # generate doxygen html headers/footers
   subprocess.check_call(['doxygen','-w','html',fn_header,fn_footer,fn_style,fn_config])
   
   # add custom css for top bar
   fp = open(fn_style,'a')
   fp.write('''
.mytitlearea {
background-color:#000000;
color:white;
font-size:14px;
}
.mytitlelabel {
padding-left:5px;
}
.mybtn {
display:inline-block;
background-color:#333333;
color: #FFFFFF;
padding: 3px 15px;
text-align: center;	
text-decoration: none;
margin-left:5px;
}
.mybtn:hover {
color:white;
background-color:#5555CC;
}
.mybtn-current {
display:inline-block;
background-color:#333388;
color: #FFFFFF;
padding: 3px 15px;
text-align: center;	
text-decoration: none;
margin-left:5px;
}
''')
   fp.close()
   
   # modify header html to add custom top bar
   headerhtml = open(fn_header).read()
   a,b,_,d,e = multisplit(headerhtml,'<!--BEGIN TITLEAREA-->','<!--END TITLEAREA-->')
   fp = open(fn_header,'w')
   fp.write(a + b)
   fp.write('''<div id="titlearea" class="mytitlearea"><span class="mytitlelabel">LEMUR Packages:</span>\n''')
   for otherpkg in sorted(pkgs):
      if otherpkg == pkg:
         fp.write('<a href="../{}/index.html" class="mybtn-current">{}</a>\n'.format(otherpkg,otherpkg))
      else:
         fp.write('<a href="../{}/index.html" class="mybtn">{}</a>\n'.format(otherpkg,otherpkg))
   fp.write('''<br style="clear:left;" /></div>\n''') # titlearea
   fp.write(d + e)
   fp.close()
   
   # modify footer html to add ReadTheDocs link
   headerhtml = open(fn_footer).read()
   a,b,c,d,e = multisplit(headerhtml,'<!--BEGIN !GENERATE_TREEVIEW-->','<!--END !GENERATE_TREEVIEW-->')
   fp = open(fn_footer,'w')
   fp.write(a + b)
   c1,c2,c3 = multisplit(c,'</small></address>')
   fp.write(c1)
   fp.write(' using <a href="https://readthedocs.org/"><img src="https://media.readthedocs.com/corporate/img/header-logo.png" alt="Read the Docs" height="31px" class="footer"/></a>')
   fp.write(c2 + c3 + d + e)
   fp.close()
   
   print('calling doxygen {} ...'.format(fn_config))
   subprocess.check_call(['doxygen',fn_config])
   
   # update tagfiles
   tagfiles[tagfile] = '../{}'.format(pkg)
   
   os.remove(fn_config)
   os.remove(fn_header)
   os.remove(fn_footer)
   os.remove(fn_style)

# throw in a toplevel html redirect to ompl_lemur
fp = open('_build/html/index.html','w')
fp.write('''<!DOCTYPE html>
<html>
<head>
<title>LEMUR Documentation</title>
<meta http-equiv="refresh" content="0; url=ompl_lemur/index.html" />
</head>
<body>
<a href="ompl_lemur/index.html">Click here</a> to be forwarded.
</body>
</html>
''')
fp.close()

# dump bogus contents.rst file fo sphinx
fp = open('contents.rst','w')
fp.close()

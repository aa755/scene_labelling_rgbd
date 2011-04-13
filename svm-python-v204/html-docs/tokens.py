import tokenize, keyword, sys, os.path

tg = tokenize.generate_tokens(file(sys.argv[1]).readline)
infile_name, lastend = sys.argv[1], (1,0)
codename = os.path.basename(infile_name)
codename = codename[:-3]

print '<html><head>'
print '<title>Code for %s</title>' % codename
print '<link href="style.css" rel="stylesheet" type="text/css">'
print '</head><body>'
print '<h1>Code for <code>%s</code></h1>' % codename
print '<pre>'

for t in tg:
    token_type,token_string,start,end,line = t
    if token_type in [4,54]: continue

    sys.stdout.write((start[0]-lastend[0])*'\n')
    numspaces = start[1]-lastend[1] if start[0]==lastend[0] else start[1]
    sys.stdout.write(numspaces*' ')
    
    lastend = end

    shtml = ''
    shtml = {53:'comment', 3:'string', 2:'number'}.get(token_type, '')
    if shtml=='comment': token_string = token_string.strip()
    if token_type == 1 and keyword.iskeyword(token_string):shtml = 'reserved'
    if shtml: sys.stdout.write('<font class="c%s">' % shtml)
    sys.stdout.write(token_string)
    if shtml: sys.stdout.write('</font>')
    
print '</pre>'

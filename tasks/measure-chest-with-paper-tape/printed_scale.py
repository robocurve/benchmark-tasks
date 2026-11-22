"""Simple readable seven-segment numerals printed at each 5 cm graduation."""
def digits(text):
 patterns={'0':'ab cdef'.replace(' ',''),'1':'bc','2':'abged','3':'abgcd','4':'fgbc','5':'afgcd','6':'afgecd','7':'abc','8':'abcdefg','9':'abfgcd'}
 seg={'a':(0,.002,.0025,.00035),'b':(.00125,.001,.00035,.002),'c':(.00125,-.001,.00035,.002),'d':(0,-.002,.0025,.00035),'e':(-.00125,-.001,.00035,.002),'f':(-.00125,.001,.00035,.002),'g':(0,0,.0025,.00035)}
 out=[]
 for j,char in enumerate(text):
  for name in patterns[char]:
   x,y,w,h=seg[name];out.append((x+(j-(len(text)-1)/2)*.0035,y+.004,w,h))
 return out

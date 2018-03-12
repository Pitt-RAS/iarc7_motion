import os

for file in os.listdir("."):
    if file.endswith(".txt"):
        with open(file) as f:
		    content = f.readlines()
		    with open('csv/{}.csv'.format(file), 'w') as o:
			    for line in content:
				    if line[0].isdigit() or line[0] == -1:
				        o.write(line)


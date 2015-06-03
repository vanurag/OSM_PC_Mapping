#transform.py

import csv

spamReader = csv.reader(open('cl.csv'))

rnew = []
for row in spamReader:
    rnew.append("{} {} {}\n".format(row[0], row[1], 0))

count = len(rnew)

newfile = open("c1new.txt", "w+")
newfile.write(str(count) + '\n')
for row in rnew:
	newfile.write(row)

newfile.close()

spamReader = csv.reader(open('cl2.csv'))

rnew = []
for row in spamReader:
    rnew.append("{} {} {}\n".format(row[0], row[1], 0))

count = len(rnew)

newfile = open("c2new.txt", "w+")
newfile.write(str(count) + '\n')
for row in rnew:
	newfile.write(row)

newfile.close()
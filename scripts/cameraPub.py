from ftplib import FTP

#Open ftp connection
ftp = FTP('192.168.42.1') 
ftp.login() 

#List the files in the current directory
print "File List:"
files = ftp.dir()
print files


#Get the readme file
ftp.cwd("/internal_000/Bebop_Drone/media")
ftp.retrlines('LIST') 

'''ftp.retrbinary('RETR Readme', gFile.write)
gFile.close()
ftp.quit()

#Print the readme file contents
print "\nReadme File Output:"
gFile = open("readme.txt", "r")
buff = gFile.read()
print buff'''

ftp.quit()
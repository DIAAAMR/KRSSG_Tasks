# Import socket module 
import socket 


def Main(): 
	# local host IP '127.0.0.1' 
	host = '127.0.0.1'

	# Define the port on which you want to connect 
	port = 12345

	s = socket.socket(socket.AF_INET,socket.SOCK_STREAM) 

	# connect to server on local computer 
	s.connect((host,port)) 

	



	print('number of elements in your array')
	n=int(input())

	print('Input the numbers you want to get sum of:')
	lst=[]

	for i in range (0,n):
		lst.append(int(input()))
	m=str(lst[0])	

	# takes an array and converts it into an string 
	for i in range (1,n):
		m=m+' '
		m=m+str(lst[i])
	m=m+ ' '+'end'
	
	#sends array to the server	
	s.send(m.encode('ascii'))	
	print('the string sent is', m)

	# messaga received from server 
	data = s.recv(1024)
	
	# print the received message  
	print('Received from the server :',int(data.decode('ascii'))) 

	s.close() 

if __name__ == '__main__': 
	Main() 

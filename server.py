#!/usr/bin/python3

"""
Student Grades Database Server Class

Ali Raza
400263872
razas21
"""

import socket
import csv
# from cryptography.fernet import Fernet
import json
import sys

class StudentGradesServer:
    HOSTNAME = "0.0.0.0"
    PORT = 50000
    RECV_BUFFER_SIZE = 1024 # Used for recv.
    MAX_CONNECTION_BACKLOG = 10 # Used for listen.
    MSG_ENCODING = "utf-8"

    def __init__(self, filename):
        self.filename = filename
        self.students_data = self.load_csv_data()
        self.create_listen_socket()
        self.process_connections_forever()


    def load_csv_data(self):
        print('Data read from CSV file:')
        with open(self.filename, newline='') as csvfile:
            for line in csvfile:
                print(line.strip())
            csvfile.seek(0)
            reader = csv.DictReader(csvfile)
            return {row['ID Number']: row for row in reader}

    def encrypt_message(self, message, key):
        # This is a placeholder for actual encryption functionality
        return message.encode(self.MSG_ENCODING)

    def create_listen_socket(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.HOSTNAME, self.PORT))
            self.socket.listen(self.MAX_CONNECTION_BACKLOG)
            print(f"\nListening for connections on port {self.PORT}.")

        except Exception as msg:
            print(msg)
            sys.exit(1)

    def process_connections_forever(self):
        try:
            while True:
                # Block while waiting for accepting incoming TCP
                # connections. When one is accepted, pass the new
                # (cloned) socket info to the connection handler
                # function. Accept returns a tuple consisting of a
                # connection reference and the remote socket address.
                self.connection_handler(self.socket.accept())
        except Exception as msg:
            print(msg)
        except KeyboardInterrupt:
            print()
        finally:
            # If something bad happens, make sure that we close the
            # socket.
            self.socket.close()
            sys.exit(1)


    def connection_handler(self, client):
        # Unpack the client socket address tuple.
        connection, address_port = client
        print("-" * 72)
        print("Connection received from {} on port {}.".format(address_port[0], address_port[1]))
        # Output the socket address.
        print(client)

        while True:
            try:
                # Receive bytes over the TCP connection. This will block
                # until "at least 1 byte or more" is available.
                recvd_bytes = connection.recv(self.RECV_BUFFER_SIZE)
            
                # If recv returns with zero bytes, the other end of the
                # TCP connection has closed (The other end is probably in
                # FIN WAIT 2 and we are in CLOSE WAIT.). If so, close the
                # server end of the connection and get the next client
                # connection.
                if len(recvd_bytes) == 0:
                    print("Closing client connection ... ")
                    connection.close()
                    break
                
                # recieved bytes are decoded then handled appropriately to get requested data
                requested_data = self.handle_bytes(recvd_bytes.decode(StudentGradesServer.MSG_ENCODING))

                if len(requested_data) == 0:
                    print("Closing client connection ... ")
                    connection.close()
                    break

                ######################### encrypt and send requested_data ###############################
                encrpyted_requested_data = self.encrypt_message(requested_data)
                connection.sendall(encrpyted_requested_data)


            except KeyboardInterrupt:
                print()
                print("Closing client connection ... ")
                connection.close()
                break

    def handle_bytes(self, data_str):
        id_number = data_str[:7]  # First 7 characters for the ID
        command = data_str[7:]    # The rest for the command

        print("Received {} command from client".format(command))

        if id_number in self.students_data:
            print("User Found.")
        else:
            print("User not found.")
            return ""
        
        student_data = self.students_data.get(id_number)

        if command == 'GG':
            # Return all grades for the requesting student
            return student_data
        elif command in ['GMA', 'GL1A', 'GL2A', 'GL3A', 'GL4A', 'GEA']:
            grades = []
            if command == 'GMA':
                grades.append(int(student_data['Midterm']))
            elif command == 'GEA':
                exams = [int(student_data[key]) for key in student_data if key.startswith('Exam')]
                return self.get_average(exams)
            else:
                # Extract lab number from command, e.g., 'GL1A' -> 1
                lab_number = int(command[2]) 
                grades.append(int(student_data[f'Lab {lab_number}']))
            return self.get_average(grades)
        else:
            return "Invalid command"

    def get_average(self, grades):
        # Calculate and return the average of a list of numeric grades
        return sum(grades) / len(grades) if grades else 0
    
        

if __name__ == '__main__':
    server = StudentGradesServer('course_grades_2024.csv')

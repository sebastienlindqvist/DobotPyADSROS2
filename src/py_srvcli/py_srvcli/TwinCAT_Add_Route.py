import pyads
import socket

TextFile='/home/vboxuser/ros2_ws/install/py_srvcli/share/py_srvcli/resource/PLC_Info.txt'

class ADS_Route():
    def __init__(self):
        super().__init__() 
        self.varialbe_list =[[],[]]
        self.resetableBool = True
        self.USERNAME, self.PASSWORD, self.TARGET_IP, self.AMSNETID = self.Read_ConnectionInfo(TextFile)
        self.LOCAL_AMSNETID, self.LOCAL_IP = self.Set_LocalAMS()
        self.Add_Route(self.LOCAL_AMSNETID,
                       self.LOCAL_IP,
                       self.TARGET_IP,
                       self.USERNAME,
                       self.PASSWORD,
                       'OOP pyads')
        self.plc=self.Open_Connection(self.AMSNETID,self.TARGET_IP)
        
    def Read_ConnectionInfo(self, Filename):
        print("\n- Opening file: "+Filename)
        print("-----------------------------------------------")
        f = open(Filename, "r")
        USERNAME=f.readline().replace("\n", "")
        print("Username: "+ repr(USERNAME))
        PASSWORD=f.readline().replace("\n", "")
        print("Password: "+ repr(PASSWORD))
        TARGET_IP=f.readline().replace("\n", "")
        print("TARGET IP: "+ repr(TARGET_IP))
        AMSNETID=f.readline().replace("\n", "")
        print("AMS NET ID: "+ repr(AMSNETID))
        print("-----------------------------------------------")
        f.close()
        return USERNAME, PASSWORD, TARGET_IP, AMSNETID
    
    def Add_Route(self, SENDER_AMS, HOSTNAME, PLC_IP, USERNAME, PASSWORD, ROUTE_NAME):
        print("\n- Adding Route to IPC")
        print("-----------------------------------------------")
        pyads.open_port()
        #pyads.set_local_address(SENDER_AMS)
        try:
            pyads.add_route_to_plc(SENDER_AMS, 
                                HOSTNAME, 
                                PLC_IP, 
                                USERNAME, 
                                PASSWORD, 
                                route_name=ROUTE_NAME
                                )
        except:
            print("Error occured when adding route")
            print("Check text file has correct information")
            print("Check Device Manager of target IPC to see if route has been made or already exists")
        pyads.close_port()

    def Open_Connection(self, TARGET_AMS_ID, TARGET_PC_IP):
        print("\n- Opening connection to IPC")
        print("-----------------------------------------------")
        try:
            plc = pyads.Connection(TARGET_AMS_ID, pyads.PORT_TC3PLC1, TARGET_PC_IP)
            plc.open()
            print ("Connection opened")
        except:
            print ("Error occured when opening connection")
        return plc
    
    def Set_LocalAMS(self):
        # Step 1: Get the local hostname.
        local_hostname = socket.gethostname()
        # Step 2: Get a list of IP addresses associated with the hostname.
        #ip_addresses = socket.gethostbyname_ex(local_hostname)[2]
        #print(ip_addresses)
        # Step 3: Filter out loopback addresses (IPs starting with "127.").
        #filtered_ips = [ip for ip in ip_addresses if not ip.startswith("127.")]
        #print(filtered_ips)
        # Step 4: Extract the first IP address (if available) from the filtered list.
        #first_ip = filtered_ips[:1]
        # Step 5: Print the obtained IP address to the console.

        #print("\nLocal IP Address found: "+first_ip[0])
        IP = "10.199.244.68"#first_ip[0] #IP address of the PC that's running this python program
        AMS_NET_ID = IP+".1.1"
        
        try:
            pyads.open_port()
            print("Setting Local AMS NET ID as: "+AMS_NET_ID)
            pyads.set_local_address(AMS_NET_ID)
            pyads.close_port()
        except:
            print("Error Ocurred -- Most likley this machine has an AMS NET ID already from TwinCAT")
            #print(pyads.get_local_address())
            AMS_NET_ID= pyads.get_local_address()
            AMS_NET_ID=str(AMS_NET_ID).split(" ",1)[1]
            print("Setting AMS_NET_ID variable is: "+str(AMS_NET_ID).split(":",1)[0])
        return AMS_NET_ID, IP
    
    def Read_Variable(self, variable):
        return self.plc.read_by_name(variable)
    
    def Write_Variable(self, variable, value):
        self.plc.write_by_name(variable, value)

    def ReadVariableList(self,Filename):
        f = open(Filename, "r")
        Continue=True
        while Continue:
            line=f.readline()     
            try:
                line=line.split(" => ")
                line[1]=line[1].split("\n")[0]
                self.varialbe_list[0].append(line[0])
                self.varialbe_list[1].append(line[1])
            except:
                Continue=False
                f.close()
        return self.varialbe_list

    def EnableFromTwinCAT(self,plc):
        check=plc.read_by_name("MAIN.ROS2pyAds")
        if check ==True and self.resetableBool==True:
            self.resetableBool=False
            return True
        elif check ==False:
            self.resetableBool=True
            return False
        else:
            return False
        
def main():

    pass

if __name__ == "__main__":
    main()
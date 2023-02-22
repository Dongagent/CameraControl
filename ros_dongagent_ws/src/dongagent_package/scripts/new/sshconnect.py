import paramiko
import os
import json


ssh_config = paramiko.SSHConfig()
try:
    ssh_config.parse(open(os.path.expanduser('~/.ssh/config')))
    labwebserver_config = ssh_config.lookup("labwebserver")
    auth_params = {}

except Exception as e:
    print(e)

# setup ssh connection
try:
    ssh_client = paramiko.client.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(
        hostname=labwebserver_config['hostname'],
        username=labwebserver_config['user'],
        key_filename=labwebserver_config['identityfile'][1],
    )
except Exception as e:
    print(e)

# get data 
# make sure you have connected to the OpenVPN
command = "cat /home/labExperiment/experiments/current/nikola_hitl_2nd/data/nikola_hitl_2nd_test1_0_.txt"
stdin,stdout,stderr=ssh_client.exec_command(command)
rawdata = stdout.readlines()

# parse the rawdata
def rawdata_extractor(d):
    return_list = []
    for dd in d:
        temp = json.loads(dd)
        return_list.append(temp["payload"])
    return return_list

payload_data = rawdata_extractor(rawdata)
print(payload_data)
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure
from pathlib import Path
import pandas as pd
import sys
import subprocess
from tqdm import tqdm
 

databaseName = "AltitudeImpact2000"
collectionPrefix = "Run"

#file contains only the connection URL
def connectdatabase(mongoFile):

    file = Path(mongoFile)
    if not file.is_file():
        raise FileNotFoundError("Could not locate Mongo file")
    fileName = ""
    with open(file, "r", encoding="utf-8") as file:
        fileName = file.read()


    client = MongoClient(fileName)
    try:
        client.admin.command('ping')
        print("MongoDB connection successful.")
    except ConnectionFailure:
        raise RuntimeError("MongoDB Connection Failed")
    return client
 
    
def uploadData(client):
    if not len(sys.argv) > 1:
        print("No file as argument")
        return
    
    csvFile = Path(sys.argv[1])
    if not csvFile.is_file():
        raise FileNotFoundError()
    
    try:
        data = pd.read_csv(sys.argv[1])
    except:
        raise RuntimeError("CSV ERROR")
    
    data = pd.DataFrame(data).iloc[::20]

    column_data = {col: data[col].tolist() for col in data.columns}
    
    database = client[databaseName]
    listOfCollection = database.list_collection_names()
    index = len(listOfCollection) + 1

    collectionName = f"{collectionPrefix}{index}"
    collection = database[collectionName]

    collection.insert_one(column_data)
    #print(f"inserted data as {collectionName}")



def simulationRun(client):
    for i in tqdm(range(300)):
        argu = f"{1000 + i * 50}"
        result = subprocess.run(["./build/rocket" , "./configs/Rocket_Config.toml" , argu ], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Return code: {result.returncode} \n output: {result.stdout} \n error output: {result.stderr}")
            raise RuntimeError("Simulation Failed")
        if result.stdout:
            print(f"Simulation #{i} Output: {result.stdout}")
        if result.stderr:
            print(f"Simulation #{i} Error: {result.stderr}")
        if result.returncode != 0:
            print(f"Simulation #{i} return code: {result.returncode}")
        uploadData(client)




if __name__ == "__main__":
    mongoFile = "mongo.txt"
    client = connectdatabase(mongoFile)
    simulationRun(client)
    





import csv

arr=[]

def openFile(name):
    with open(name, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in spamreader:
            arr.append(row)
            print(arr)

def djikstra(file):
    # alg

def bfs(file):
    # alg

def dfs(file):
    # alg


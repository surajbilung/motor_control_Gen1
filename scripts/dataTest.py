#!/home/pi/vEnvs/rosPy/bin/python

'''
Test to build dataFrames
'''
import pandas

#Home: E:\\Documents\\Scripts\\Robot-Arm\\testCoordinates.csv
#Work: U:\\Documents\\GitCode\\Robot-Arm\\testCoordinates.csv

#dataFrame = pandas.DataFrame(pandas.read_csv\
#    ("U:\\Documents\\GitCode\\Robot-Arm\\testCoordinates.csv", sep=","))
#dataFrame = dataFrame.drop(columns=["BASE", "MAIN", "SEC", "TOOL"])
#dataList = [tuple(x) for x in dataFrame.values]

class dataManager(object):
    '''
    This class will consolidate the uses of pandas and numpy to handle the list
    of coordinates for a given arm sequence. It will generate dataFrames, convert
    to lists of tuples, print individual coordinates, etc.
    '''
    def __init__(self, filePath):
        self.dataFrame = pandas.DataFrame(pandas.read_csv(filePath, sep=','))

    def dropCols(self, dropList):
        '''
        Drops any unnecessary columns from the DataFrame. Pass this function a
        list of column names to remove them from DataFrame.
        '''
        self.dataFrame = self.dataFrame.drop(columns=dropList)

    def frameToList(self, dataFrame):
        '''
        Typically run as the final step of managing data frame. It will convert
        the pandas dataframe into a list of tuples, which will serve as the list
        of coordinates fed to the nodes.
        '''
        return [tuple(x) for x in dataFrame.values]

dataFrame = dataManager("U:\\Documents\\GitCode\\Robot-Arm\\testCoordinates.csv")
dataFrame.dropCols(["BASE", "MAIN", "SEC", "TOOL"])
#dataList = dataFrame.frameToList

print(dataFrame)

#dataFrame = pandas.DataFrame(pandas.read_csv\
#    ("U:\\Documents\\GitCode\\Robot-Arm\\testCoordinates.csv", sep=","))
#dataFrame = dataFrame.drop(columns=["BASE", "MAIN", "SEC", "TOOL"])
#dataList = [tuple(x) for x in dataFrame.values]

#def genDataFrame(filePath):
#   '''
#    Turns a CSV File into a Pandas DataFrame.
#    '''
#    return pandas.DataFrame(pandas.read_csv(filePath, sep=","))

#def dropCols(dataFrame, dropList):
#    '''
#    Drops any unnecessary columns from the DataFrame. Pass this function a
#   list of column names to remove them from DataFrame.
#    '''
#    return dataFrame.drop(columns=dropList)

#def frameToList(dataFrame):
#    '''
#    Typically run as the final step of managing data frame. It will convert
#   the pandas dataframe into a list of tuples, which will serve as the list
#    of coordinates fed to the nodes.
#    '''
#    return [tuple(x) for x in dataFrame.values]

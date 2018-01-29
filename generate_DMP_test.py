from dmp_experiments.Python import train_dmp

file1 = "/media/nathaniel/Data/Google Drive/haptic devices for home-based stroke rehab/code/python/Main/RecordedTrials/Trial1_DH.csv"
file2 = "/media/nathaniel/Data/Google Drive/haptic devices for home-based stroke rehab/code/python/Main/RecordedTrials/Trial2_DH.csv"

names = [ file1, file2 ]

train_dmp.batch_train_dmp(names,100,0.01,"DMP")

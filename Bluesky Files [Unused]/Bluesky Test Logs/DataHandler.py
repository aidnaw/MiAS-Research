def datahandler():
    ''' Function for retrieving data from BlueSky log files. Outputs a dictionary with keys = aircraft number
    and items = subdictionaries. Subdictionary keys = measurement labels and subdictionary items = lists of 
    corresponding data. Everything in output is in string format.
    - Aidan Wallace, University of MD College Park, 2020 '''

    # Prompt user to specify file for analysis
    # f = input('Input BlueSky log file for data analysis:')

    # Read in log file
    with open('TESTLOG_00_Custom Log_20201031_08-02-07.log') as file:
        rawtag = file.readline()
        rawlabels = file.readline()
        rawdata = file.readlines()

    # Get log tag and data labels
    tag = rawtag.strip('# ')
    labels = rawlabels.strip('# ').replace(',',' ').split()
    print(f'This log is tagged: {tag}')
    print(f'This log contains information on {labels}')
    

    # Convert data entries to lists
    formatted_data = []
    for line in rawdata:
        formatted_data.append(line.strip('\n').replace(',',' ').split())

    # Find how many aircraft are on the log
    acno = 1
    initialentry = formatted_data[0]
    for entry in formatted_data[1::]:
        if entry[0] == initialentry[0]: acno += 1 #If initial time is the same, it's a different aircraft
        else: break
    print(f'This log contains {acno} aircraft.')

    # Parse data into the logs for each individual aircraft
    acdata_list = []
    for i in range(acno):
         acdata_list.append(formatted_data[i::acno])

    # Create empty nested dictionary for each aircraft and its measured values
    data = dict()
    for i in range(acno):
        data.setdefault(f'Aircraft {i+1}', dict())

    # Populate the data dictionary
    for acno, aircraft in enumerate(data.keys()):
        for varno, label in enumerate(labels):
            data[aircraft].setdefault(label, [item[varno] for item in acdata_list[acno]])

    return data

data = datahandler()
print(data['Aircraft 1']['lat'])
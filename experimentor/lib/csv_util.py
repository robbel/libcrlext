import sys

def getCSVColumns(in_csv, start, end):
    return [row[start:end] for row in in_csv]
def getCSVRows(in_csv, start, end):
    return in_csv[start:end]

def readCSV(path):
    return [line.strip().split('\t') for line in open(path).readlines()]

def writeCSV(csv, path):
    out = open(path, 'w')
    for line in csv:
        for box in line:
            if box:
                out.write(str(box)+'\t')
            else:
                out.write('\t')
        out.write('\n')
    out.close()

def averageCSVs(in_csvs):
    avg_csv = []
    lengths = [len(csv) for csv in in_csvs]
    length = lengths[0]
    for l in lengths:
        if l != length:
            raise "in_csvs not all same size"
    for i in range(length):
        lines = [csv[i] for csv in in_csvs]
        line_lengths = [len(line) for line in lines]
        line_length = line_lengths[0]
        for l in line_lengths:
            if l != line_length:
                raise "in_csvs not all same size"
        avg_line = []
        for j in range(line_length):
            try:
                vals = [int(line[j]) for line in lines]
                avg = 1.0*sum(vals)/len(vals)
                avg_line.append(str(avg))
            except:
                avg_line.append(in_csvs[0][i][j])
        avg_csv.append(avg_line)
    return avg_csv

    
def mean_confidence_interval(data, confidence=0.95):
    from numpy import mean, array, sqrt
    import scipy.stats  
    a = 1.0*array(data)
    n = len(a)
    m, se = mean(a), scipy.stats.stderr(a)
    # calls the inverse CDF of the Student's t distribution
    h = se * scipy.stats.t._ppf((1+confidence)/2., n-1)
    return h
    
def errorCSVs(in_csvs):
    err_csv = []
    lengths = [len(csv) for csv in in_csvs]
    length = lengths[0]
    for l in lengths:
        if l != length:
            raise "in_csvs not all same size"
    for i in range(length):
        lines = [csv[i] for csv in in_csvs]
        line_lengths = [len(line) for line in lines]
        line_length = line_lengths[0]
        for l in line_lengths:
            if l != line_length:
                raise "in_csvs not all same size"
        err_line = []
        for j in range(line_length):
            vals = []
            try:
                vals = [float(line[j]) for line in lines]
            except:
                err_line.append(in_csvs[0][i][j])
                continue
            try:
                err = mean_confidence_interval(vals)
                err_line.append(str(err))
            except:
                err_line.append(str(-1))
        err_csv.append(err_line)
    return err_csv
    
 

def augmentCSVs(in_csvs):
    out_csv = []
    widths = [max(line_width) for line_width in [[len(line) for line in csv] for csv in in_csvs]]
    lengths = [len(csv) for csv in in_csvs]
    length = max(lengths)
    for row in range(length):
        line = []
        for i in range(len(in_csvs)):
            line_width = 0
            if row < len(in_csvs[i]):
                line_width = len(in_csvs[i][row])
            for col in range(line_width):
                line.append(in_csvs[i][row][col])
            line += [None]*(widths[i]-line_width)
        out_csv.append(line)
        
    return out_csv

if __name__ == "__main__":
    writeCSV(augmentCSVs([readCSV(sys.argv[1]), readCSV(sys.argv[2])]), '/dev/stdout')
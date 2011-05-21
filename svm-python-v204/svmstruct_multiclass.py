"""A module for SVM^python for multiclass learning."""

import svmapi
from numpy import *
global NUM_CLASSES
NUM_CLASSES=0

def read_examples(filename, sparm):
    """Parses an input file into an example sequence."""
    global NUM_CLASSES 
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    # Open the file and read each example.
    for line in file(filename):
        # Get rid of comments.
        if line.find('#'): line = line[:line.find('#')]
        tokens = line.split()
        # If the line is empty, who cares?
        if not tokens: continue
        NUM_CLASSES = int(tokens[0])
        # Get the target.
        target = int(tokens[1])
        # Get the features.
        tokens = [tuple(t.split(':')) for t in tokens[3:]]
        features = [(0,1)]+[(int(k),float(v)) for k,v in tokens]
        # Add the example to the list
        examples.append((svmapi.Sparse(features), target))
    # Print out some very useful statistics.
    print len(examples),'examples read'
    return examples

def init_model(sample, sm, sparm):
    """Store the number of features and classes in the model."""
    # Note that these features will be stored in the model and written
    # when it comes time to write the model to a file, and restored in
    # the classifier when reading the model from the file.
    global NUM_CLASSES
    sm.num_features = max(max(x) for x,y in sample)[0]+1
    sm.num_classes = NUM_CLASSES
    print sm.num_classes*sm.num_features
    sm.size_psi = sm.num_features * sm.num_classes
    print 'size_psi set to',sm.size_psi

def classification_score(x,y,sm,sparm):
    """Return an example, label pair discriminant score."""
    # Utilize the svmapi.Model convenience method 'classify'.
    return sm.svm_model.classify(psi(x,y,sm,sparm))

def classify_example(x, sm, sparm):
    """Returns the classification of an example 'x'."""
    # Construct the discriminant-label pairs.
    scores = [(classification_score(x,c,sm,sparm), c)
              for c in xrange(1,sm.num_classes+1)]
    # Return the label with the max discriminant value.
    return max(scores)[1]

def find_most_violated_constraint(x, y, sm, sparm):
    """Returns the most violated constraint for example (x,y)."""
    # Similar, but include the loss.
    scores = [(classification_score(x,c,sm,sparm)+loss(y,c,sparm), c)
              for c in xrange(1,sm.num_classes+1)]
    return max(scores)[1]

def psi(x, y, sm, sparm):
    """Returns the combined feature vector Psi(x,y)."""
    # Just increment the feature index to the appropriate stack position.
    offset = sm.num_features * (y-1)
    pvec = svmapi.Sparse([(k+offset,v) for k,v in x], kernel_id=y)
    return pvec

def loss(y, ybar, sparm):
    # We use zero-one loss.
    if y==ybar: return 0
    return 1


def eval_prediction(exnum, (x, y), ypred, sm, sparm, teststats):
    """Accumulate statistics about a single training example.

    Allows accumulated statistics regarding how well the predicted
    label ypred for pattern x matches the true label y.  The first
    time this function is called teststats is None.  This function's
    return value will be passed along to the next call to
    eval_prediction.  After all test predictions are made, the last
    value returned will be passed along to print_testing_stats.

    On the first call, that is, when exnum==0, teststats==None.  The
    default behavior is that the function does nothing."""
    if exnum==0: teststats = []
    teststats.append((y,ypred));
    return teststats

def print_testing_stats(sample, sm, sparm, teststats):
    """Print statistics once classification has finished.

    This is called after all test predictions are made to allow the
    display of any summary statistics that have been accumulated in
    the teststats object through use of the eval_prediction function.

    The default behavior is that nothing is printed."""


    aggConfusionMatrix=zeros((sm.num_classes,sm.num_classes), dtype='i')
    prec = []
    recall = []
    ltc = []
    lpc=[]
    for t in teststats:
        aggConfusionMatrix[t[1]-1,t[0]-1]+=1
    tc = 0;
    for i  in xrange(0,sm.num_classes):
        sum = 0;
        for j in xrange(0,sm.num_classes):
            sum+= aggConfusionMatrix[i,j]
        tc += sum
        if (sum !=0):
            prec.append(aggConfusionMatrix[i,i]*100.0/sum);
        else:
            prec.append(0);
        lpc.append(sum);
    for i  in xrange(0,sm.num_classes):
        sum = 0;
        for j in xrange(0,sm.num_classes):
            sum+= aggConfusionMatrix[j,i]
        if (sum !=0):
            recall.append(aggConfusionMatrix[i,i]*100.0/sum);
        else:
            recall.append(0);
        ltc.append(sum);
    for i in xrange(0,sm.num_classes):
        print "label ", i+1 , " prec: ", prec[i], " recall: ", recall[i], " tp: ", aggConfusionMatrix[i,i], " tc: ",ltc[i], " pc: ",lpc[i]
    match = 0
    for i in xrange(0,sm.num_classes):
        match += aggConfusionMatrix[i,i]

    print "prec: ", match*100.0/tc , "recall: ",match*100.0/tc, "tp: ",match, " pc: ", tc, "tc: ",tc

    print "confusion matrix:"
    print aggConfusionMatrix;
    savetxt('conf.txt',aggConfusionMatrix);

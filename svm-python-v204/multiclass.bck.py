"""A module for SVM^python for multiclass learning."""

# Thomas Finley, tfinley@gmail.com

import svmapi

def read_examples(filename, sparm):
    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    # Open the file and read each example.
    for line in file(filename):
        # Get rid of comments.
        if line.find('#'): line = line[:line.find('#')]
        tokens = line.split()
        # If the line is empty, who cares?
        if not tokens: continue
        # Get the target.
        target = int(tokens[0])
        # Get the features.
        tokens = [t.split(':') for t in tokens[1:]]
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
    sm.num_features = max(max(x) for x,y in sample)[0]+1
    sm.num_classes = max(y for x,y in sample)
    sm.size_psi = sm.num_features
    #print 'size_psi set to',sm.size_psi

thecount = 0
def classification_score(x,y,sm,sparm):
    """Return an example, label pair discriminant score."""
    # Utilize the svmapi.Model convenience method 'classify'.
    score = sm.svm_model.classify(psi(x,y,sm,sparm))
    global thecount
    thecount += 1
    if (sum(abs(w) for w in sm.w)):
        import pdb; pdb.set_trace()
    return score

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
    ybar = max(scores)[1]
    #print y, ybar
    return max(scores)[1]

def psi(x, y, sm, sparm):
    """Returns the combined feature vector Psi(x,y)."""
    # Just increment the feature index to the appropriate stack position.
    #vecness = [(k,v) for k,v in x]
    pvec = svmapi.Sparse(x, kernel_id=y)
    #print list(sm.w)
    #print pveca
    #import pdb; pdb.set_trace()
    return svmapi.Document([pvec])

def loss(y, ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    return 100.0*int(y != ybar)

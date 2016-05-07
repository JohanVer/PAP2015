#include <iostream>
#include <dlib/svm.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace dlib;
using namespace cv;

typedef matrix<double, 3, 1> sample_type;

template <class T>
std::vector<T> getSubset( std::vector<T> &in , size_t subset_size){
    std::vector<T> out;
    for(size_t i = 0; i < subset_size; i++){
        out.push_back(in.at(i));
    }
    return out;
}


void extractTrainingDataFromImages(const cv::Mat &org, const cv::Mat &labels_pic, std::vector<sample_type> &samples, std::vector<double> &labels){

    size_t positive_counter = 0;
    size_t negative_counter = 0;

    // Get positive examples

    for(size_t r = 0; r < org.rows; r++){
        for(size_t c = 0; c < org.cols; c++){
            cv::Vec3b pix = org.at<cv::Vec3b>(r, c);
            cv::Vec3b pix_label = labels_pic.at<cv::Vec3b>(r, c);
            sample_type new_sample;

            if(pix_label[1] == 0xff){
                new_sample(0) = pix[0];
                new_sample(1) = pix[1];
                new_sample(2) = pix[2];
                samples.push_back(new_sample);
                labels.push_back(+1);
                positive_counter++;
            }
        }
    }

    // Get negative examples

    for(size_t r = 0; r < org.rows; r++){
        if(negative_counter >= positive_counter){
            break;
        }
        for(size_t c = 0; c < org.cols; c++){
            cv::Vec3b pix = org.at<cv::Vec3b>(r, c);
            cv::Vec3b pix_label = labels_pic.at<cv::Vec3b>(r, c);
            sample_type new_sample;

            if(pix_label[2] == 0xff){
                new_sample(0) = pix[0];
                new_sample(1) = pix[1];
                new_sample(2) = pix[2];
                samples.push_back(new_sample);
                labels.push_back(-1);
                negative_counter++;
                if(negative_counter >= positive_counter){
                    break;
                }
            }
        }
    }
}

int main()
{

    typedef radial_basis_kernel<sample_type> kernel_type;

    std::vector<sample_type> samples_full;
    std::vector<double> labels_full;

    // Now let's put some data into our samples and labels objects.  We do this by looping
    // over a bunch of points and labeling them according to their distance from the
    // origin.

    cv::Mat pic = cv::imread(string(getenv("PAPRESOURCES")) + "training_data/filternull_org.png",CV_LOAD_IMAGE_COLOR);
    cv::Mat pic_label = cv::imread(string(getenv("PAPRESOURCES")) + "training_data/filternull_label.png",CV_LOAD_IMAGE_COLOR);


    cv::imshow("test", pic);
    extractTrainingDataFromImages(pic, pic_label, samples_full, labels_full);

    vector_normalizer<sample_type> normalizer;
    // let the normalizer learn the mean and standard deviation of the samples
    normalizer.train(samples_full);
    // now normalize each sample
    for (unsigned long i = 0; i < samples_full.size(); ++i)
        samples_full[i] = normalizer(samples_full[i]);

    randomize_samples(samples_full, labels_full);

    std::vector<sample_type> samples;
    std::vector<double> labels;

    //samples = samples_full;
    //labels = labels_full;

    samples = getSubset(samples_full, 5000);
    labels = getSubset(labels_full, 5000);

    // The nu parameter has a maximum value that is dependent on the ratio of the +1 to -1
    // labels in the training data.  This function finds that value.
    const double max_nu = maximum_nu(labels);

    // here we make an instance of the svm_nu_trainer object that uses our kernel type.
    svm_nu_trainer<kernel_type> trainer;

    /*
    // Now we loop over some different nu and gamma values to see how good they are.  Note
    // that this is a very simple way to try out a few possible parameter choices.  You
    // should look at the model_selection_ex.cpp program for examples of more sophisticated
    // strategies for determining good parameter choices.
    cout << "doing cross validation" << endl;
    for (double gamma = 0.00001; gamma <= 1; gamma *= 5)
    {
        for (double nu = 0.00001; nu < max_nu; nu *= 5)
        {
            // tell the trainer the parameters we want to use
            trainer.set_kernel(kernel_type(gamma));
            trainer.set_nu(nu);

            cout << "gamma: " << gamma << "    nu: " << nu;
            // Print out the cross validation accuracy for 3-fold cross validation using
            // the current gamma and nu.  cross_validate_trainer() returns a row vector.
            // The first element of the vector is the fraction of +1 training examples
            // correctly classified and the second number is the fraction of -1 training
            // examples correctly classified.
            cout << "     cross validation accuracy: " << cross_validate_trainer(trainer, samples, labels, 3);
        }
    }
    */

    trainer.set_kernel(kernel_type(0.15625));
    trainer.set_nu(0.15625);

    // Nonlinear svm function

    typedef decision_function<kernel_type> dec_funct_type;
    typedef normalized_function<dec_funct_type> funct_type;

    funct_type learned_function;

    learned_function.normalizer = normalizer;  // save normalization information
    learned_function.function = trainer.train(samples, labels); // perform the actual SVM training and save the results

    cout << "\nnumber of support vectors in our learned_function is "
         << learned_function.function.basis_vectors.size() << endl;

    learned_function.function = reduced2(trainer,3).train(samples, labels);

    serialize(string(getenv("PAPRESOURCES")) + "trained_functions/red_svm_function.dat") << learned_function;


    // Probabilistic nonlinear svm function

    typedef probabilistic_decision_function<kernel_type> probabilistic_funct_type;
    typedef normalized_function<probabilistic_funct_type> pfunct_type;

    pfunct_type learned_pfunct;
    learned_pfunct.normalizer = normalizer;
    learned_pfunct.function = train_probabilistic_decision_function(trainer, samples, labels, 3);

    cout << "\ncross validation accuracy with only 3 support vectors: "
         << cross_validate_trainer(reduced2(trainer,3), samples, labels, 3);

    // And similarly for the probabilistic_decision_function:
    learned_pfunct.function = train_probabilistic_decision_function(reduced2(trainer,3), samples, labels, 3);

    serialize(string(getenv("PAPRESOURCES")) + "trained_functions/red_prob_svm_function.dat") << learned_pfunct;


    // Linear SVM
    typedef linear_kernel<sample_type> kernel_type_lin;
    typedef dlib::decision_function<kernel_type_lin> lin_func_type;
    typedef normalized_function<lin_func_type> lin_funct_type;

    svm_c_linear_trainer<kernel_type_lin> linear_trainer;

    linear_trainer.set_c(10);

    lin_funct_type df;
    df.normalizer = normalizer;
    df.function = linear_trainer.train(samples, labels);

    cout << "\nLinear Cross validation : "
         << cross_validate_trainer(linear_trainer, samples, labels, 3);

    serialize(string(getenv("PAPRESOURCES")) + "trained_functions/linear_svm_function.dat") << df;
}



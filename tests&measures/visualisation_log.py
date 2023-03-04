from matplotlib import pyplot as plt
from tools import *
import numpy as np

mission_name = 're'

# Ouvrir le fichier et voir ce qu'on lit
f = open("log_files/log_%s.txt" % mission_name, 'r')
lines = f.readlines()

p = 5
submissions = []
data=None

# Lire les data et les stocker dans un array
for no_line in range(len(lines)):
    line = lines[no_line]
    if line == '\n':
        if p == 0:
            submission[2] = no_line-len(submissions)*3 
            submissions.append(submission)
        submission = ['0', no_line-len(submissions)*3, 0, []] # name, begin_submission, end_submission
        p = 1
    elif p == 1:
        submission_title = line[14:-2]
        #print(submission_title)
        submission[0] = submission_title
        p = 2
    elif p == 2:
        keys = line.split(' ')[:-1]  # Valeurs loguées pendant la mission
        #print(keys)
        submission[3] = keys
        p = 0
    else:
        measure = lines[no_line].split(' ')[:-1]
        measure=np.array([float(mes) for mes in measure])
        data = np.vstack((data, np.array(measure).reshape(1, len(keys)))) if data is not None else np.array(measure).reshape(1, len(keys))

submission[2] = data.shape[0] 
submissions.append(submission)

#print(len(submissions))

fig_mag, ax_mag = plt.subplots(3,1)
fig_mag.suptitle('Magneto')
fig_acc, ax_acc = plt.subplots(3,1)
fig_acc.suptitle('Accelero')
fig_rpm, ax_rpm = plt.subplots(2,1)
fig_rpm.suptitle('Rpm')
fig_d_psi, ax_d_psi = plt.subplots()
fig_d_psi.suptitle('Delta PSI')
fig_th, ax_th = plt.subplots(2,1)
fig_th.suptitle('Temperature motors')


def plot_submission(submission, plot_mag=True, plot_acc=True, plot_rpm=True, plot_d_psi=True, plot_th=False):
    title = submission[0]
    BEGIN_SUBMISSION = submission[1]
    END_SUBMISSION = submission[2]
    keys == submission[3]

    time = np.linspace(BEGIN_SUBMISSION*0.5,END_SUBMISSION*0.5,END_SUBMISSION-BEGIN_SUBMISSION) #data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index('time')].T #

    # Afficher ce qui nous intéresse
    if plot_mag:
        if 'mx' in keys:
            mag_keys = ['mx', 'my', 'mz']
            for i in range(len(mag_keys)):
                ax_mag[i].plot(time, data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index(mag_keys[i])].T)
                ax_acc[i].set_title(mag_keys[i])

    if plot_acc:
        if 'ax' in keys:
            acc_keys = ['ax', 'ay', 'az']
            for i in range(len(acc_keys)):
                ax_acc[i].plot(time, data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index(acc_keys[i])].T)
                ax_acc[i].set_title(acc_keys[i])

    if plot_rpm:
        if 'rpmL' in keys:
            rpm_keys = ['rpmL', 'rpmL_bar', 'rpmR', 'rpmR_bar']
            tit_keys = ['Rpm Left engine', 'Rpm Right engine']
            for i in range(2):
                ax_rpm[i].plot(time, data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index(rpm_keys[2*i])].T)#, label=rpm_keys[2*i])
                ax_rpm[i].plot(time, data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index(rpm_keys[2*i+1])].T, color='red')#, label=rpm_keys[2*i+1])
                #plt.locator_params(axis='y', nbins=10)
                #plt.locator_params(axis='x', nbins=50)
                ax_rpm[i].set_title(tit_keys[i])
                #ax_rpm[i].legend()

    if plot_d_psi:
        if 'd_PSI' in keys:
            ax_d_psi.plot(time, data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index('d_PSI')].T)
            #plt.locator_params(axis='x', nbins=50)
            ax_d_psi.set_title('d_PSI')

    if plot_th:
        if 'thL' in keys:
            th_keys = ['thL', 'thR']
            for i in range(len(th_keys)):
                ax_th[i].plot(time, data[BEGIN_SUBMISSION:END_SUBMISSION, keys.index(th_keys[i])].T)
                #plt.locator_params(axis='x', nbins=50)
                ax_th[i].set_title(th_keys[i])



def plot_Allmission(plot_mag=True, plot_acc=True, plot_rpm=True, plot_d_psi=True, plot_th=False):  # all submissions must be of the same type
    for submission in submissions:
        plot_submission(submission, plot_mag=True, plot_acc=True, plot_rpm=True, plot_d_psi=True, plot_th=True)

AXES= [ax_mag[0], ax_mag[1], ax_mag[2], ax_acc[0], ax_acc[1], ax_acc[2], ax_rpm[0], ax_rpm[1], ax_d_psi, ax_th[0], ax_th[1]] 
def fix_axes():
    for ax in AXES:
        start,end=ax.get_xlim()
        ax.xaxis.set_ticks(np.round(np.linspace(start,end,15)))

        start,end=ax.get_ylim()
        ax.yaxis.set_ticks(np.linspace(start,end,10))

if __name__=="__main__":
    #plot_Allmission()
    plot_submission(submissions[-1])
    #fix_axes()
    plt.show()


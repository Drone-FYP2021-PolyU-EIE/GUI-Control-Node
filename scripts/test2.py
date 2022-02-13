import tkinter as tk

class widget:
    def __init__(self,master):
        # vcmd = (master.register(self.validate),
                # '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.master = master
        self.entryNumbers = tk.Entry(master,justify = tk.CENTER)
        self.entryNumbers.insert(0, "5")
        self.entryNumbers.grid(row = 0,column = 0,columnspan =2,sticky="EW")
        self.createEntriesButton = tk.Button(master,text = "Create Entries",command = self.createEntries)
        self.createEntriesButton.grid(row = 1, column = 0,columnspan = 2,sticky="EW")

    def createEntries(self):
        self.entryNumbers.grid_forget()
        self.createEntriesButton.grid_forget()

        self.entries = []
        self.entryLabels = []

        vcmd = self.master.register(self.validateEntry)

        for i in range(int(self.entryNumbers.get())):
            self.entryLabels.append(tk.Label(self.master,text = "Row {}".format(i)))
            self.entryLabels[-1].grid(row = i,column = 0)
            self.entries.append(tk.Entry(self.master, validatecommand=(vcmd,'%P',i)))
            self.entries[-1].grid(row = i,column = 1)

        self.addEntriesButton = tk.Button(self.master,text = "Add Entries",command = self.addEntry)
        self.addEntriesButton.grid(row = i+1, column = 0,columnspan = 2,sticky="EW")

    def addEntry(self): 
        count = len(self.entries)

        vcmd = self.master.register(self.validateEntry)

        self.entryLabels.append(tk.Label(self.master,text = "Row {}".format(count)))
        self.entryLabels[-1].grid(row = count+1,column = 0)
        self.entries.append(tk.Entry(self.master, validatecommand=(vcmd,'%P',count)))
        self.entries[-1].grid(row = count+1,column = 1)
        self.addEntriesButton.grid(row = count+2, column = 0,columnspan = 2,sticky="EW")

    def validateEntry(self,P,row):
        if P != row:
            return True
        else:
            return False

root1=tk.Tk()
widget(root1)
root1.mainloop()
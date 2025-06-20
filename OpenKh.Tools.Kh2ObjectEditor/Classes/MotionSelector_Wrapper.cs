using OpenKh.Kh2;
using OpenKh.Tools.Kh2ObjectEditor.Services;

namespace OpenKh.Tools.Kh2ObjectEditor.Classes
{
    public class MotionSelector_Wrapper
    {
        public int Index { get; set; }
        public BinaryArchive.Entry Entry { get; set; }
        public string Name { get; set; }
        public bool IsDummy { get { return Name.Contains("DUMM") || LinkedSubfile.Length == 0; } }
        public byte[] LinkedSubfile
        {
            get
            {
                if(Entry.Link == -1)
                {
                    return new byte[0];
                }

                return MsetService.Instance.MsetBinarc.Subfiles[Entry.Link];
            }
        }

        public MotionSelector_Wrapper(int index, BinaryArchive.Entry entry)
        {
            Index = index;
            Entry = entry;
            setName();
        }
        public void setName()
        {
            int motionIndex = (MsetService.Instance.MsetBinarc.MSetType == BinaryArchive.MotionsetType.Player) ? Index / 4 : Index;
            Name = "[" + Index + "] " + Entry.Name + " [" + (MotionSet.MotionName)motionIndex + "]";
             
            // Tags
            if (Entry.Type == BinaryArchive.EntryType.Motionset)
                Name += "<SET>";
            if (LinkedSubfile.Length == 0)
                Name += "<DUMMY>";
        }
    }
}

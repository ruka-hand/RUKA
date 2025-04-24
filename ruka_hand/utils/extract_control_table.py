import pandas as pd
from bs4 import BeautifulSoup

from ruka_hand.utils.file_ops import get_repo_root

# controlTable.html taken from https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#model-information (just the EEPROM and RAM <table> elements)
with open(f"{get_repo_root()}/ruka_hand/utils/assets/controlTable.html") as fp:
    soup = BeautifulSoup(fp, "html.parser")

data = []

# go through each table and take all row contents and put them in df
tables = soup.find_all("tbody")
for table in tables:
    for tr in table.find_all("tr"):
        row = [td.text for td in tr.find_all("td")]
        data.append(row)
df_controlTable = pd.DataFrame(
    data,
    columns=[
        "Address",
        "Size(Byte)",
        "Data Name",
        "Access",
        "Initial Value",
        "Range",
        "Unit",
    ],
)

# drop indirect addresses, if they are needed comment this out
df_controlTable.drop(df_controlTable.tail(14).index, inplace=True)

# Change dtype to int to when accessing address and size they can easily be used. The rest of columns have missing vals or are strings
df_controlTable["Address"] = df_controlTable["Address"].astype(int)
df_controlTable["Size(Byte)"] = df_controlTable["Size(Byte)"].astype(int)

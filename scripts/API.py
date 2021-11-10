#!/usr/bin/env python3

from database import Database, ShoppingListTable, ItemTable
from nltk import PorterStemmer

stem = PorterStemmer()
DB = Database()
shopping_list_table = ShoppingListTable(DB)
item_table = ItemTable(DB)

class API():
    @staticmethod
    def add_item(item, quantity):
        return item_table.add_quantity("default", stem.stem(item), item, int(quantity))

    @staticmethod
    def remove_item(item, quantity):
        if item_table.get("default", stem.stem(item)) is None:
            raise ValueError()

        if quantity == "all":
            return item_table.delete("default", stem.stem(item))
        return item_table.remove_quantity("default", stem.stem(item), int(quantity)) 

    @staticmethod
    def show_list():
        return item_table.all()

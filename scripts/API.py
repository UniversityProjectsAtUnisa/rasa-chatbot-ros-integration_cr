#!/usr/bin/env python

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
        return item_table.remove_quantity("default", stem.stem(item), int(quantity))  

    @staticmethod
    def show_list():
        return item_table.all()
